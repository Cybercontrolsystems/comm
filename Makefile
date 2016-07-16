# Fairly generic cross-compliation makefile for simple programs
CC=$(CROSSTOOL)/$(ARM)/bin/gcc
NAME=comm

all: $(NAME)
	$(CROSSTOOL)/$(ARM)/bin/strip $(NAME)

$(NAME): $(NAME).c
