PREFIX = /usr/local

boids: boids.c termbox.h
	$(CC) $(CFLAGS) -O2 boids.c -o boids -lm

.PHONY: clean
clean:
	$(RM) boids

.PHONY: install
install: boids
	mkdir -p $(PREFIX)/bin
	install boids $(PREFIX)/bin/boids

.PHONY: uninstall
uninstall:
	$(RM) $(PREFIX)/bin/boids

