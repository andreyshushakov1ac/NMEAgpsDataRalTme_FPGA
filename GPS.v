/*

отображает геоданные в режиме (!) реального времени (!) 
на 1ой строке широта долгота, на 2ой время.

Данные должны по протоколу NMEA
(с gps-приёмника)

*/

module GPS
(
////////////////////////////////////
// for rs232 in/out

input clk,
input  rx, // линия передачи данных
output reg err, //сигнал ошибки 

// for rs232 in/out
////////////////////////////////////
// for lcd in/out

output reg [7:0] Din,
//RS: 0-команды, 1- data _ RW: 0- write, 1- read
output reg RS, RW,
output reg EN //enable
);
// for lcd in/out
////////////////////////////////////
// for lcd reg,param,init

reg [4:0] a1; //вспомогательный регистр. Равен кол-ву принимаемых символов
reg [5:0] state1, nextstate1; // состояния автомата
//50MHz * 200ms = 10000000 импульсов. Задержка между операциями 200мс
reg [23:0] count;

parameter init =3'b001; //Инициализация: Шина 8 бит, 2 строки
parameter up =3'b010; //включение дисплея
parameter clear =3'b011; //очистка содержимого
parameter addr_incr =3'b100; //Задает направление перемещения курсора и создаёт сдвиг после каждого введённого символа
//parameter cursor_incr =3'b101; //Сдвиг курсора
parameter data_writing =3'b110; //запись в дисплей

initial 
begin
	a1=0;
	state1 = init;
	nextstate1 = init;
	RW = 0; 
	RS=0; 
	EN = 0; 
end

// for lcd reg,param,init
////////////////////////////////////
// for rs232 reg,param,init

parameter idle = 2'b00; //режим ожидания
parameter startbit = 2'b01; //стартбит
parameter databit = 2'b10; //режим передачи данных
parameter stopbit = 2'b11; //стопбит
//делитель счётчика: 50Мгц(импульсов/с)/9600(бит/с)=5208(импульсов/с)
//обеспечивает нужную скорость передачи бит
parameter divider = 13'd5208; 

reg [3:0] state, nextstate; // текущее и следующее состояние автомата
reg [3:0] bitcount; // cчётчик битов
reg [12:0] tickcount; // счётчик импульсов
reg [7:0] rd; //регистр принятого байт 
//вспомогательный рег
reg [4:0] a;
//для аски кодов 2ух цифр северной широты
reg [15:0] shirota;
//для аски кодов 3x цифр восточной долготы
reg [23:0] dolgota;
//для аски кодов 2ух цифр часов
reg [15:0] hour;
//для аски кодов 2ух цифр минут
reg [15:0] min;
//для аски кодов 2ух цифр секунд
reg [15:0] sec;
// Знаки направлений широты и долготы
reg [7:0] ZnakDolgoti, ZnakShiroti;

initial begin
	state = idle; //текущее состояние - ожидание
	rd = 0; 
	nextstate = idle; //следующее состояние - ожидание
	bitcount = 4'b0000; // счётчик битов обнулён
	tickcount = 0; // счётчик импульсов обнулён
	err = 1'b0; a=0; 
	sec=0; min=0; hour=0; shirota=0; dolgota=0; 
	ZnakDolgoti=0; ZnakShiroti=0;
end

// for rs232 reg,param,init
////////////////////////////////////
// 1ый из 2х  always-блоков: приёма данных по rs232

always @(posedge clk)
begin
	state = nextstate; //переходим в следующее состоние 
	
	// Пишем действия при нахождении в каждом из состояний автомата
	case (state) 
	
		idle: 
		begin 
			rd = 8'h00;
			tickcount = 0;
			bitcount = 0;
			err = 1'b0;
			if (rx == 1'b0) 
				nextstate = startbit;
		end
		
		startbit:
		begin
			tickcount = tickcount + 1; //на импульс продвигаемся
			if ( tickcount == divider/2)//12'd2604 )
			begin
				if (rx == 1'b0)
				begin
					err = 1'b0;
				end else
				begin 
					err = 1'b1;
					nextstate = idle;
				end 
			end else 	
					if (tickcount == divider)
					begin
						tickcount = 0;
						nextstate = databit;
					end
		end
		
		databit:
		begin
			tickcount = tickcount + 1; //на импульс продвигаемся
			if ( tickcount == divider/2 )
			begin
				rd[bitcount] = rx; 
				bitcount = bitcount + 1; //начинаем считать информационные биты
			end else			
			if (tickcount == divider) 
			//если все 5208 импульсов на бит прошли, то
			begin
				tickcount = 0; // счётчик обнуляем
				if (bitcount == 4'b1000)
				//если все 8 бит данных прошли, то
				begin
					bitcount = 0; //счётчик информационных битов обнуляем
					nextstate = stopbit; //переходим в режим стопбита
				end
				else nextstate = databit;
			end
		end	
			
		stopbit:
		begin	
			tickcount = tickcount + 1;
			if ( tickcount == divider/2 )
			begin
				if (rx == 1'b1)
				begin
					err = 1'b0;
					
////////////////////////////////////
//сохранение необходимых данных в регистры					 					

					if (rd==8'b00100100) //"$"
						a<=0;
					
					else
					
					if (rd==8'b01000111 & a==0) //"G" для получения GPGGA
						a<=19;
					
					else
					
					if (rd==8'b01010000 & a==19) //"P" для получения GPGGA
						a<=20;
					
					else
					
					if (rd==8'b01000111 & a==20) //"G" для получения GPGGA
						a<=21;
					
					else
					
					if (rd==8'b01000111 & a==21) //"G" для получения GPGGA
						a<=22;
					
					else
					
					if (rd==8'b01000001 & a==22) //"A" для получения GPGGA
						a<=1;
					
					else
					
					if (rd==8'b00101100 & a==1) // 44 = ","
						a <= 2; //перая запятая	
					
					else
		
					if (a==2)
					begin
						a<=3; //первая цифра часов
						hour[7:0] = rd;
					end
					
					else
					
					if (a==3)
					begin
						a<=4; //вторая цифра часов
						hour[15:8] = rd;
					end
					
					else
					
					if (a==4)
					begin
						a<=5; //первая цифра минут
						min[7:0] = rd;
					end
					
					else
					
					if (a==5)
					begin
						a<=6; //вторая цифра минут
						min[15:8] = rd;
					end
					
					else 
					
					if (a==6)
					begin
						a<=7; //первая цифра секунд
						sec[7:0] = rd;
					end
					
					else 
					
					if (a==7)
					begin
						a<=8; //вторая цифра секунд
						sec[15:8] = rd;
					end	
						
					else
					
					if (rd==8'b00101100 & a==8) // 44 = ","
						a <= 9; //вторая запятая
					
					else
					
					if (a==9)
					begin
						a<=10; //первая цифра широты
						shirota[7:0] = rd;
					end
					
					else
					
					if (a==10)
					begin
						a<=11; //вторая цифра широты
						shirota[15:8] = rd;
					end
					
					else
					
					if (rd==8'b00101100 & a==11) // 44 = ","
						a <= 12; //третья запятая	
						
					else
					
					// знак направления широты
					if (a==12)
					begin 
						a<=13; 
						ZnakShiroti = rd;
					end
					
					if (rd==8'b00101100 & a==13) // 44 = ","
						a <= 14; //4ая запятая
					
					else
					
					if (a==14)
					begin
						a<=15; //первая цифра долготы
						dolgota[7:0] = rd;
					end
					
					else
					
					if (a==15)
					begin
						a<=16; //вторая цифра долготы
						dolgota[15:8] = rd;
					end
					
					else
					
					if (a==16)
					begin
						a<=17; 
						dolgota[23:16] = rd;//третья цифра долготы
					end		
					
					if (rd==8'b00101100 & a==17) // 44 = ","
					a <= 18; //5ая запятая
					
					else

					// знак направления долготы
					if (a==18)
					begin  
						ZnakDolgoti = rd;
						a<=0; //возвращаем назад для синхронизации
					end	
					
//сохранение необходимых данных в регистры						
////////////////////////////////////
// дописываем состояние стопбита
					
					nextstate = stopbit;
				end else
				begin
					err = 1'b1;
					rd=0;
					nextstate = idle;
				end
			end else
			if (tickcount == divider) 
			begin
				tickcount = 0;
				nextstate = idle;
			end else
				nextstate = stopbit;
		end
// дописываем состояние стопбита
////////////////////////////////////		
	default: state = idle;
	endcase;
end

////////////////////////////////////
// 2ой из 2х always-блоков для вывода на lcd

always @(posedge clk)
begin 

state1 = nextstate1;

	case (state1)
		
		init: 
		begin
			EN = 1;
			Din = 8'b00111000;
			count = count + 1;
			if (count==1000000)
			begin
				EN = 0;
				count=0;
				nextstate1 = clear;
			end
			else 
				nextstate1 = state1;
		end
		
		
		clear:
		begin
			EN = 1;
			Din = 8'b00000001;
			count = count + 1;
			if (count==1000000)
			begin
				EN = 0;
				count=0;
				nextstate1 = addr_incr;
			end
			else 
				nextstate1 = state1;
		end
		
	
		addr_incr:
		begin
			EN = 1;
			Din = 8'b00000110;
			count = count + 1;
			if (count==1000000)
			begin
				EN = 0;
				count=0;
				nextstate1 = up;
			end
			else 
				nextstate1 = state1;
		end
		
	
		up:
		begin
			EN = 1;
			Din = 8'b00001100;
			count = count + 1;
			if (count==1000000)
			begin
				EN = 0;
				count=0;
				nextstate1 = data_writing;//cursor_incr;
			end
			else 
				nextstate1 = state1;
		end
		
		/*
		cursor_incr:
		begin
			EN = 1;
			Din = 8'b00010100;
			count = count + 1;
			if (count==10000000)
			begin
				EN = 0;
				count=0;
				nextstate1 = data_writing;
			end
			else 
				nextstate1 = state;
		end
		*/
		
		data_writing:
		begin
			RS = 1;
			// ставим курсор в начало первой строки (для 2х и более обходов always-блокa)
			if (a1==0) 
			begin
				RS=0;
				EN = 1;
				Din = 8'b10000000;
				count = count + 1;
				if (count==125000)
				begin
					RS=1;
					EN = 0;
					count=0;
					a1<=1;
				end
				else 
					nextstate1 = state1;	
			end	
			
			//запись первой цифры широты
			if (a1==1)
			begin
				EN = 1;
				Din = shirota[7:0];
				count = count + 1;
				if (count==125000)
				begin
					EN = 0;
					count=0;
					a1 <= 2;
				end
				else 
					nextstate1 = state1;
			end	
			
			//запись 2 цифры широты
			if (a1==2)
			begin
				EN = 1;
				Din = shirota[15:8];
				count = count + 1;
				if (count==125000)
				begin
					EN = 0;
					count=0;
					a1<=3;
				end
				else 
					nextstate1 = state1;			
			end
			
			//знак градуса
			if (a1==3)
			begin
				EN = 1;
				Din = 8'b11011111;
				count = count + 1;
				if (count==125000)
				begin
					EN = 0;
					count=0;
					a1<=4;
				end
				else 
					nextstate1 = state1;			
			end
			
			// напрвление широты 
			if (a1==4)
			begin
				EN = 1;
				Din = ZnakShiroti; 
				count = count + 1;
				if (count==125000)
				begin
					EN = 0;
					count=0;
					a1<=5;
				end
				else 
					nextstate1 = state1;
			end
			
			// " " 
			if (a1==5)
			begin
				EN = 1;
				Din = 8'b00100000;
				count = count + 1;
				if (count==125000)
				begin
					EN = 0;
					count=0;
					a1<=6;
				end
				else 
					nextstate1 = state1;
			end	
				
			//запись первой цифры долготы 
			if (a1==6)
			begin
				EN = 1;
				Din = dolgota[7:0];
				count = count + 1;
				if (count==125000)
				begin
					EN = 0;
					count=0;
					a1<=7;
				end
				else 
					nextstate1 = state1;
			end	
			
			//запись второй цифры долготы 
			if (a1==7)
			begin
				EN = 1;
				Din = dolgota[15:8];
				count = count + 1;
				if (count==125000)
				begin
					EN = 0;
					count=0;
					a1<=8;
				end
				else 
					nextstate1 = state1;
			end	
			
			//запись третей цифры долготы 
			if (a1==8)
			begin
				EN = 1;
				Din = dolgota[23:16];
				count = count + 1;
				if (count==125000)
				begin
					EN = 0;
					count=0;
					a1<=9;
				end
				else 
					nextstate1 = state1;
			end
			
			//знак градуса
			if (a1==9)
			begin
				EN = 1;
				Din = 8'b11011111;
				count = count + 1;
				if (count==125000)
				begin
					EN = 0;
					count=0;
					a1<=10;
				end
				else 
					nextstate1 = state1;			
			end
			
			//" направление долготы
			if (a1==10)
			begin
				EN = 1;
				Din = ZnakDolgoti;
				count = count + 1;
				if (count==125000)
				begin
					EN = 0;
					count=0;
					a1<=11;
				end
				else 
					nextstate1 = state1;
			end		
			
			//Перевод в нижнюю строку 
			if (a1==11) 
			begin
				RS=0;
				EN = 1;
				Din = 8'b11000000;
				count = count + 1;
				if (count==125000)
				begin
					RS=1;
					EN = 0;
					count=0;
					a1<=12;
				end
				else 
					nextstate1 = state1;	
			end	
		
			// запись первой цифры часов
			if (a1==12)
			begin
				EN = 1; 
				Din = hour[7:0];
				count = count + 1;
				if (count==125000)
				begin
					EN = 0;
					count=0;
					a1<=13;
				end
				else 
					nextstate1 = state1;
			end	
			
			// запись второй цифры часов
			if (a1==13)
			begin
				EN = 1;
				Din = hour[15:8];
				count = count + 1;
				if (count==125000)
				begin
					EN = 0;
					count=0;
					a1<=14;
				end
				else 
					nextstate1 = state1;
			end	
			
			// ":" 1
			if (a1==14)
			begin
				EN = 1;
				Din = 8'b00111010;
				count = count + 1;
				if (count==125000)
				begin
					EN = 0;
					count=0;
					a1<=15;
				end
				else 
					nextstate1 = state1;
			end
			
			// запись первой цифры минут
			if (a1==15)
			begin
				EN = 1;
				Din = min[7:0];
				count = count + 1;
				if (count==125000)
				begin
					EN = 0;
					count=0;
					a1<=16;
				end
				else 
					nextstate1 = state1;
			end

			// запись второй цифры минут
			if (a1==16)
			begin
				EN = 1;
				Din = min[15:8];
				count = count + 1;
				if (count==125000)
				begin
					EN = 0;
					count=0;
					a1<=17;
				end
				else 
					nextstate1 = state1;
			end
			
			// ":" 2
			if (a1==17)
			begin
				EN = 1;
				Din = 8'b00111010;
				count = count + 1;
				if (count==125000)
				begin
					EN = 0;
					count=0;
					a1<=18;
				end
				else 
					nextstate1 = state1;
			end
			
			// запись первой цифры секунд
			if (a1==18)
			begin
				EN = 1;
				Din = sec[7:0];
				count = count + 1;
				if (count==125000)
				begin
					EN = 0;
					count=0;
					a1<=19;
				end
				else 
					nextstate1 = state1;
			end
			
			// запись второй цифры секунд
			if (a1==19)
			begin
				EN = 1;
				Din = sec[15:8];
				count = count + 1;
				if (count==125000)
				begin
					EN = 0;
					count=0;
					a1<=0; //Возвращаемся в начало!!!
				end
				else 
					nextstate1 = state1;
			end

		end
	endcase;	
end

// 2ой из 2х always-блоков для вывода на lcd
//////////////////////////////

endmodule 




