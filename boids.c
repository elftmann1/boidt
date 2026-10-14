/*
 * boids - a flock of boids in the terminal
 *
 * A rewrite of lavat (a lava-lamp metaball animation) that replaces the
 * metaball physics with Craig Reynolds' classic boid algorithm. Each boid
 * steers by three local rules - separation, alignment and cohesion - which
 * together produce emergent flocking, like a school of fish or a murmuration
 * of birds.
 *
 * Rendering is done with TermBox so it works in any terminal that supports
 * 256-colour output. Press 'h' for help, 'q' to quit.
 */

#define TB_IMPL
#define TB_LIB_OPTS
#define TB_OPT_TRUECOLOR

#include "termbox.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- tunables ----------------------------------------------------------*/
#define TRAIL          5      /* cells of fading trail behind each boid head */
#define PERCEPTION     6.0f   /* neighbourhood radius (cells)               */
#define SEP_RADIUS     2.5f   /* separation radius (cells)                  */

/* steering weights */
#define W_SEPARATION   1.6f
#define W_ALIGNMENT    1.0f
#define W_COHESION     0.9f

/* speed limits */
#define MAX_SPEED      2.4f   /* cells / frame                              */
#define MIN_SPEED      0.8f
#define MAX_FORCE      0.12f  /* how hard a boid can steer per frame       */

/* ---- palette ------------------------------------------------------------*/
/* each entry is a TermBox colour index (0..255). The head of every boid is
 * drawn in one of these; the fading trail uses a single dim colour.         */
static const unsigned short PALETTE[] = {
    39, 45, 51, 87, 117, 153, 185, 209, 225
};
#define NPAL (int)(sizeof(PALETTE) / sizeof(PALETTE[0]))
static const unsigned short DIM_COLOR = 235; /* dark grey for the trail   */

/* ---- state -------------------------------------------------------------*/
typedef struct {
    float x, y;    /* position (cells)        */
    float vx, vy;  /* velocity                */
} Boid;

static Boid *flock = NULL;
static int   nboids = 0;

/* per-cell render buffers, sized to the terminal */
static char            *cell_char = NULL;
static unsigned short  *cell_fg   = NULL;
static unsigned short  *cell_bg   = NULL;
static int             *life      = NULL; /* remaining trail life per cell */

static int W = 0, H = 0;
static int running = 1;
static int paused = 0;

/* ---- helpers -----------------------------------------------------------*/
static void steer_limit(float *ax, float *ay)
{
    float m = sqrtf(*ax * *ax + *ay * *ay);
    if (m > MAX_FORCE) {
        *ax *= MAX_FORCE / m;
        *ay *= MAX_FORCE / m;
    }
}

/* one boid's contribution to the three flocking forces */
static void flock_forces(int i, float *fx, float *fy)
{
    Boid *b = &flock[i];
    float sep_x = 0, sep_y = 0;
    float ali_x = 0, ali_y = 0;
    float coh_x = 0, coh_y = 0;
    int n_sep = 0, n_nei = 0;

    for (int j = 0; j < nboids; j++) {
        if (j == i) continue;
        Boid *o = &flock[j];
        float dx = b->x - o->x;
        float dy = b->y - o->y;
        float d2 = dx * dx + dy * dy;
        float r = PERCEPTION;
        if (d2 > r * r) continue;

        float d = sqrtf(d2);
        if (d < 1e-4f) d = 1e-4f;

        /* alignment: match neighbours' heading */
        ali_x += o->vx;
        ali_y += o->vy;
        /* cohesion: steer toward local centre of mass */
        coh_x += o->x;
        coh_y += o->y;
        n_nei++;

        if (d < SEP_RADIUS) {
            /* separation: push away, weighted by closeness */
            sep_x += dx / d;
            sep_y += dy / d;
            n_sep++;
        }
    }

    float ax = 0, ay = 0;

    if (n_nei > 0) {
        /* alignment */
        ali_x /= n_nei; ali_y /= n_nei;
        float am = sqrtf(ali_x * ali_x + ali_y * ali_y);
        if (am > 1e-4f) { ali_x /= am; ali_y /= am; }
        ax += W_ALIGNMENT * (ali_x - b->vx / (am > 1e-4f ? am : 1.0f));
        ay += W_ALIGNMENT * (ali_y - b->vy / (am > 1e-4f ? am : 1.0f));

        /* cohesion */
        coh_x = coh_x / n_nei - b->x;
        coh_y = coh_y / n_nei - b->y;
        float cm = sqrtf(coh_x * coh_x + coh_y * coh_y);
        if (cm > 1e-4f) { coh_x /= cm; coh_y /= cm; }
        ax += W_COHESION * coh_x;
        ay += W_COHESION * coh_y;
    }

    if (n_sep > 0) {
        float sm = sqrtf(sep_x * sep_x + sep_y * sep_y);
        if (sm > 1e-4f) { sep_x /= sm; sep_y /= sm; }
        ax += W_SEPARATION * sep_x;
        ay += W_SEPARATION * sep_y;
    }

    steer_limit(&ax, &ay);
    *fx = ax;
    *fy = ay;
}

static void step_flock(void)
{
    float fx[nboids], fy[nboids];
    for (int i = 0; i < nboids; i++)
        flock_forces(i, &fx[i], &fy[i]);

    for (int i = 0; i < nboids; i++) {
        Boid *b = &flock[i];
        b->vx += fx[i];
        b->vy += fy[i];

        float sp = sqrtf(b->vx * b->vx + b->vy * b->vy);
        if (sp > MAX_SPEED) { b->vx *= MAX_SPEED / sp; b->vy *= MAX_SPEED / sp; }
        else if (sp < MIN_SPEED && sp > 1e-4f) {
            b->vx *= MIN_SPEED / sp;
            b->vy *= MIN_SPEED / sp;
        }

        b->x += b->vx;
        b->y += b->vy;

        /* wrap around the edges */
        if (b->x < 0) b->x += W; else if (b->x >= W) b->x -= W;
        if (b->y < 0) b->y += H; else if (b->y >= H) b->y -= H;
    }
}

/* ---- rendering ---------------------------------------------------------*/
static void clear_cells(void)
{
    for (int i = 0; i < W * H; i++) {
        cell_char[i] = ' ';
        cell_fg[i]   = 255;
        cell_bg[i]   = TB_DEFAULT;
        life[i]      = 0;
    }
}

static void draw_cell(int x, int y, char ch, unsigned short fg)
{
    if (x < 0 || x >= W || y < 0 || y >= H) return;
    int i = y * W + x;
    cell_char[i] = ch;
    cell_fg[i]   = fg;
    life[i]      = TRAIL;
}

static void render(void)
{
    clear_cells();

    for (int i = 0; i < nboids; i++) {
        Boid *b = &flock[i];
        int cx = (int)b->x, cy = (int)b->y;
        unsigned short col = PALETTE[i % NPAL];

        /* draw a short trail opposite to the velocity */
        for (int t = 1; t <= TRAIL; t++) {
            float tx = b->x - b->vx * t * 0.6f;
            float ty = b->y - b->vy * t * 0.6f;
            draw_cell((int)tx, (int)ty, '.', DIM_COLOR);
        }

        /* head: pick a glyph by heading */
        char ch = '~';
        if (b->vx > 0.5f)       ch = '>';
        else if (b->vx < -0.5f) ch = '<';
        else if (b->vy > 0.5f)  ch = 'v';
        else if (b->vy < -0.5f) ch = '^';
        draw_cell(cx, cy, ch, col);
    }

    /* fade existing trails one step */
    for (int i = 0; i < W * H; i++) {
        if (life[i] > 0) life[i]--;
        if (life[i] == 0 && cell_char[i] != ' ') {
            /* keep the glyph but dim it so it reads as a fading tail */
            cell_fg[i] = DIM_COLOR;
        }
    }

    tb_clear();
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            int i = y * W + x;
            if (cell_char[i] == ' ') continue;
            tb_set_cell(x, y, (uint32_t)cell_char[i], cell_fg[i], TB_DEFAULT);
        }
    tb_present();
}

/* ---- lifecycle ---------------------------------------------------------*/
static void alloc_cells(void)
{
    free(cell_char); free(cell_fg); free(cell_bg); free(life);
    cell_char = calloc(W * H, sizeof(char));
    cell_fg   = calloc(W * H, sizeof(unsigned short));
    cell_bg   = calloc(W * H, sizeof(unsigned short));
    life      = calloc(W * H, sizeof(int));
}

static void spawn_flock(void)
{
    free(flock);
    flock = malloc(sizeof(Boid) * nboids);
    for (int i = 0; i < nboids; i++) {
        flock[i].x = rand() % W;
        flock[i].y = rand() % H;
        float a = ((float)rand() / RAND_MAX) * 6.28318f;
        float s = MIN_SPEED + ((float)rand() / RAND_MAX) * (MAX_SPEED - MIN_SPEED);
        flock[i].vx = cosf(a) * s;
        flock[i].vy = sinf(a) * s;
    }
}

static void on_resize(void)
{
    int w, h;
    w = tb_width(); h = tb_height();;
    if (w != W || h != H) {
        W = w; H = h;
        alloc_cells();
        if (flock) spawn_flock();
    }
}

/* ---- main loop ---------------------------------------------------------*/
int main(int argc, char **argv)
{
    int opt_count = 120;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-n") && i + 1 < argc)
            opt_count = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-h")) {
            printf("usage: boids [-n count]\n");
            printf("  -n count   number of boids (default %d)\n", opt_count);
            return 0;
        }
    }
    nboids = opt_count < 1 ? 1 : opt_count;

    if (tb_init() == -1) {
        fprintf(stderr, "Error: Failed to initialize termbox.\n");
        return 1;
    }

    W = tb_width();
    H = tb_height();
    alloc_cells();
    spawn_flock();

    while (running) {
        on_resize();

        struct tb_event ev;
        
        /* 
         * Non-blocking check for input (timeout = 0 ms).
         * In classic termbox, this returns TB_EVENT_KEY (>0) on keypress.
         * In termbox2, this returns TB_OK (0) and populates ev.type.
         */
        int ret = tb_peek_event(&ev, 0);

        if (ret > 0 || ret == 0) {
            /* Support both classic Termbox (ev.type or ret) and Termbox2 */
            int type = ev.type ? ev.type : ret;

            if (type == TB_EVENT_KEY) {
                if (ev.ch != 0) {
                    switch (ev.ch) {
                    case 'q':
                        running = 0;
                        break;
                    case 'p':     /* pause */
                        paused = !paused;
                        break;
                    }
                } else if (ev.key == TB_KEY_ESC) {
                    running = 0;
                }
            }
        }

        if (!paused) step_flock();
        render();

        /* Explicit delay drives the 30 FPS update loop */
        usleep(33000); /* 33ms */
    }

    tb_shutdown();
    return 0;
}