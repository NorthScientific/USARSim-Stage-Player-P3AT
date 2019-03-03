INSTALLDIR	=~/local/
INCLUDEDIR	=~/local/include/player-2.2/
PLAYER_LIBS =-L ~/local/lib/ -lplayerc++

CPP        = g++

OBJ        = obj/
SRC        = src/

CFLAGS	= -g -fPIC -I$(INCLUDEDIR)

all:$(OBJ)libUsarsim.so
install: $(OBJ)libUsarsim.so
	@echo "installing libUsarsim.so into $(INSTALLDIR)lib/"
	@cp $(OBJ)libUsarsim.so $(INSTALLDIR)lib/libUsarsim.so
uninstall: 
	@echo "uninstalling libUsarsim.so"
	@rm $(INSTALLDIR)lib/libUsarsim.so
clean:
	@echo "Cleaning up UsarSim"
	@rm -f $(OBJ)*.o
	@rm -f $(OBJ)*.a
	@rm -f $(OBJ)*.so 

USARSIMOBJS = $(OBJ)us_bot.o $(OBJ)us_bumper.o\
			$(OBJ)us_fakelocalize.o $(OBJ)us_fiducial.o $(OBJ)us_ir.o\
			$(OBJ)us_laser.o $(OBJ)us_laser3d.o $(OBJ)us_parser.o\
			$(OBJ)us_position.o $(OBJ)us_position3d.o $(OBJ)us_ptz.o\
			$(OBJ)us_ranger.o $(OBJ)us_sonar.o

$(OBJ)libUsarsim.so: $(USARSIMOBJS)
	@echo -e "Linking USARSIM Library...\n"
	@$(CPP) -o  $(OBJ)libUsarsim.so -fPIC -shared $(USARSIMOBJS)

$(OBJ)us_bot.o: $(SRC)us_bot.*
	@echo "Compiling us_bot..."
	@$(CPP) $(CFLAGS) -c -o $(OBJ)us_bot.o $(SRC)us_bot.cc
	
$(OBJ)us_bumper.o: $(SRC)us_bot.* $(SRC)us_bumper.*
	@echo "Compiling us_bumper..."
	@$(CPP) $(CFLAGS) -c -o $(OBJ)us_bumper.o $(SRC)us_bumper.cc
	
$(OBJ)us_fakelocalize.o: $(SRC)us_bot.* $(SRC)us_fakelocalize.*
	@echo "Compiling us_fakelocalize..."
	@$(CPP) $(CFLAGS) -c -o $(OBJ)us_fakelocalize.o $(SRC)us_fakelocalize.cc
	
$(OBJ)us_fiducial.o: $(SRC)us_bot.* $(SRC)us_fiducial.*
	@echo "Compiling us_fiducial..."
	@$(CPP) $(CFLAGS) -c -o $(OBJ)us_fiducial.o $(SRC)us_fiducial.cc

$(OBJ)us_ir.o: $(SRC)us_bot.* $(SRC)us_ir.*
	@echo "Compiling us_ir..."
	@$(CPP) $(CFLAGS) -c -o $(OBJ)us_ir.o $(SRC)us_ir.cc
	
$(OBJ)us_laser.o: $(SRC)us_bot.* $(SRC)us_laser.*
	@echo "Compiling us_laser..."
	@$(CPP) $(CFLAGS) -c -o $(OBJ)us_laser.o $(SRC)us_laser.cc
	
$(OBJ)us_laser3d.o: $(SRC)us_bot.* $(SRC)us_laser3d.*
	@echo "Compiling us_laser3d..."
	@$(CPP) $(CFLAGS) -c -o $(OBJ)us_laser3d.o $(SRC)us_laser3d.cc
	
$(OBJ)us_parser.o: $(SRC)us_parser.*
	@echo "Compiling us_parser..."
	@$(CPP) $(CFLAGS) -c -o $(OBJ)us_parser.o $(SRC)us_parser.cc
	
$(OBJ)us_position.o: $(SRC)us_bot.* $(SRC)us_position.*
	@echo "Compiling us_position..."
	@$(CPP) $(CFLAGS) -c -o $(OBJ)us_position.o $(SRC)us_position.cc
	
$(OBJ)us_position3d.o: $(SRC)us_bot.* $(SRC)us_position3d.*
	@echo "Compiling us_position3d..."
	@$(CPP) $(CFLAGS) -c -o $(OBJ)us_position3d.o $(SRC)us_position3d.cc
	
$(OBJ)us_ptz.o: $(SRC)us_bot.* $(SRC)us_ptz.*
	@echo "Compiling us_ptz..."
	@$(CPP) $(CFLAGS) -c -o $(OBJ)us_ptz.o $(SRC)us_ptz.cc
	
$(OBJ)us_ranger.o: $(SRC)us_bot.* $(SRC)us_ranger.*
	@echo "Compiling us_ranger..."
	@$(CPP) $(CFLAGS) -c -o $(OBJ)us_ranger.o $(SRC)us_ranger.cc
	
$(OBJ)us_sonar.o: $(SRC)us_bot.* $(SRC)us_sonar.*
	@echo "Compiling us_sonar..."
	@$(CPP) $(CFLAGS) -c -o $(OBJ)us_sonar.o $(SRC)us_sonar.cc
	
