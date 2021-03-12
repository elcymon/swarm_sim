comm_models:    main.o  comm_models.o
	g++ -o  comm_models main.o  comm_models.o

main.o : main.cpp comm_models.hh
	g++ -c -std=c++11 main.cpp

comm_models.o : comm_models.cc comm_models.o
	g++ -c -std=c++11 comm_models.cc

clean : 
	rm main.o comm_models.o
