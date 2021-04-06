CC=clang++
flags=-std=c++17 -stdlib=libc++ -Wall -Wextra -Wreorder-ctor

hash_gen_parse_test: hash_gen_parse_test.o icosahedron.o triangle.o phex.o point3.o
	mkdir -p builds
	$(CC) $(flags) -o builds/hash_gen_parse_test hash_gen_parse_test.o icosahedron.o triangle.o phex.o point3.o
	make clean

hash_test: hash_test.o icosahedron.o triangle.o phex.o point3.o
	mkdir -p builds
	$(CC) $(flags) -o builds/hash_test hash_test.o icosahedron.o triangle.o phex.o point3.o
	make clean

hash_gen_parse_test.o: src/enums.hpp src/icosahedron.hpp src/point3.hpp
	$(CC) $(flags) -c test/hash_gen_parse_test.cpp

hash_test.o: src/enums.hpp src/icosahedron.hpp src/point3.hpp
	$(CC) $(flags) -c test/hash_test.cpp

icosahedron.o: src/enums.hpp src/point3.hpp src/triangle.hpp
	$(CC) $(flags) -c src/icosahedron.cpp

triangle.o: src/enums.hpp src/point3.hpp
	$(CC) $(flags) -c src/triangle.cpp

phex.o: src/icosahedron.hpp src/point3.hpp
	$(CC) $(flags) -c src/phex.cpp

point3.o: src/constants.hpp src/enums.hpp
	$(CC) $(flags) -c src/point3.cpp

clean:
	rm -f *.o