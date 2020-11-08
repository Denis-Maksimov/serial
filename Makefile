





a.out: ./Serial.c ./main.c
	@gcc ./Serial.c ./main.c -Wall
	@size a.out

clean:
	rm a.out
