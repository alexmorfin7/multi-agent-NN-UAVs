/*
 * Envio_datos.pde
 *
 *  Created on: 05/12/2017
 *      Author: Orlando
 */
static void Telemetria(){
  uint16_t aux;

  //Llenado del buffer de escritura
  _buffertx[0]='R';
  _buffertx[1]=(roll*57.2)+90;
  _buffertx[2]=(pitch*57.2)+90;
  _buffertx[3]=(yaw*57.2)+90;
  _buffertx[4]=(gyrox*57.2)+90;
  _buffertx[5]=(gyroy*57.2)+90;
  _buffertx[6]=(gyroz*57.2)+90;

  aux=(pos_x+50)*100;
   _buffertx[7]=aux >> 8;
   _buffertx[8]=aux & 0x00FF;
  aux=(pos_y+50)*100;
   _buffertx[9]=aux>>8;
   _buffertx[10]=aux & 0x00FF;
   aux=(pos_z+5)*100;
   _buffertx[11]=aux >> 8;
   _buffertx[12]=aux & 0x00FF;
   _buffertx[13]=6;
   _buffertx[14]='\n';

//Escribir el buffer en el puerto serial correspondiente
   hal.uartC->write(_buffertx,15);
   //Limpiar buffer de escritura
   _buffertx[0]=0;
   _buffertx[1]=0;
   _buffertx[2]=0;
   _buffertx[3]=0;
   _buffertx[4]=0;
   _buffertx[5]=0;
   _buffertx[6]=0;
   _buffertx[7]=0;
   _buffertx[8]=0;
   _buffertx[9]=0;
   _buffertx[10]=0;
   _buffertx[11]=0;
   _buffertx[12]=0;
   _buffertx[13]=0;
   _buffertx[14]=0;



}



