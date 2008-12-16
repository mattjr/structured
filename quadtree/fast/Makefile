       CC      = gcc
       OUT     = -o
       OPT     = -O3
       DEBUG   = -g 
       
       .SUFFIXES: .o .cpp
       .cpp.o: 
		$(CC) -c $(OPT) -o $@ $<

       OBJECTS = Quadtree.o \
                 Quadnode.o \
                 Mytree.o \
                 Mynode.o

       LIBS    = -lm

       default: demo

       demo: $(OBJECTS) demo.o
		$(CC) $(OUT) demo $(OBJECTS) demo.o $(LIBS)

       
