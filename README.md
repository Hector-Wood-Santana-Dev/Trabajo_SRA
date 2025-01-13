# Trabajo_SRA
Repositorio para la realización del trabajo de curso de Sistemas Robóticos Autónomos, el cual trata de una competición usando los robots Lego Mindstorms.

## Objetivo
El robot debe aparcar entre dos latas separadas 40cm entre sí, y situadas tal y como se muestra en la siguiente figura (dimensiones aproximadas), utilizando como referencia la pizarra existente en el laboratorio.

![alt text](/Assets/image.png)

## Desarrollo de la Competición
El robot parte desde la zona izquierda del escenario. Su posición inicial no es conocida, pero se sabe que se encontrará a menos de 25cm de la línea negra, y mirando en dirección a las latas con una desviación máxima de 25 grados.

El robot deberá emitir un sonido y cambiar el color de los leds antes de realizar ningún movimiento. En ese instante se arrancará un cronómetro que medirá el tiempo invertido en completar la misión.

Deberá desplazarse hasta la zona de aparcamiento sin tirar las latas, emitiendo un sonido y cambiando el color de los leds cuando haya finalizado. En ese momento, se detendrá el cronómetro. Para que el aparcamiento se considere perfecto, debe quedar una rueda a cada lado de la línea, sin pisarla.

Puntos de aparcamiento: una rueda a cada lado 10 ptos, una rueda a un lado y la otra pisando la línea 3 ptos, las dos ruedas pisando la línea 2 ptos.