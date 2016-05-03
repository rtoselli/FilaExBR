# FilaExBR

Código e hardware para uma filamentadora simples de baixo custo, com controle pid para a temperatura e PWM para a velocidade do motor.
O codigo está em constante desenvolvimento, acompanhe o repositório para as últimas versões.

Todo o conteúdo desse repositório está disponivel sob a licença GNU - GPL, e não me responsabilizo por qualquer dano pelo o uso impróprio dos arquivos contidos aqui.

-----

Hardware and code for a simple filament extruder control board with low cost and high power, PID and PWM for motor/temperature control. 
The code evolves fast, keep an eye for new versions. 

All files in this repository are under the GNU - GPL license, use at your own risk, i am not responsible for any harm done by the misuse of the files contained here.


# Board 

![FilaExBR](/Board/filaex.png?raw=true "FilaExBR")

The top layer routing represent jumpers, no need for a double sided PCB!
You can see a basic functions video here : https://www.youtube.com/watch?v=0D0GXgb3E8Y

#Bill of materials
Pin Header 1x2 - 3 

Terminal block 5mm - 3

D1 - 1n4007
 
Linear Regulator 7805 - 1 

LED5MM - 2 
                                                        
Switch SPDT - 15A @ 12V - 1 

IRLB3813 N-Channel Enhancement MOSFET (HEXFET); 30V; 100A~ - 2 
                              
R1 - 220R

R2 - 220R

R3 - 47k

Trimpot 300R

ALPS rotary Encoder EC12E with switch - 1
                                                                             
ARDUINO NANO - 1

LCD Display HD44780LCD 16X02 - 1
                                                 