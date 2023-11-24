/*
 * Data.pde
 *
 *  Created on: 29/11/2017
 *      Author: Orlando
 */
static uint16_t log_num;   //Dataflash


//DEfiniendo el primer paquete "pose"


struct PACKED log_Pose{
    LOG_PACKET_HEADER;
    float    alt_barof;
    float    Roll;
    float    Pitch;
    float    Yaw;
    float    z_pos;
    float    x_vel;
    float    y_vel;
    float    z_vel;
    float    x_pos;
    float    y_pos;
    float    giroz;
    float    girox;
    float    giroy;
};


struct PACKED log_Control {
    LOG_PACKET_HEADER;
    float  time_ms;
    float  u_z;
    float  tau_theta;
    float  tau_phi;
    float  tau_psi;
    float  alturaR;//tau_x
    float  velR;//tau_y
    float  alturaF;//suma_phi
    float  velF;
};

struct PACKED log_Errors {
    LOG_PACKET_HEADER;
    uint32_t   time_ms;
    float   x_error;
    float   y_error;
    float   z_error;
    float   voltaje;
    float   corriente;
    int   switch1;
    int   switch2;
    int     gas;
    float   z_des; // aqui me quede
    float   x_des;
    float   y_des;
};


//Definiendo el encabezado como aparecerá en el binario

static const struct LogStructure log_structure[] PROGMEM = {
         LOG_COMMON_STRUCTURES,
         {LOG_POSE_MSG, sizeof(log_Pose),
          "1", "fffffffffffff", "a_bar,ROLL,PITCH,YAW,Z_POS,V_X,V_Y,V_Z,X_POS,Y_POS,G_Z,G_X,G_Y"},
          { LOG_CONTROL_MSG, sizeof(log_Control),
          "2", "fffffffff", "T_MS,UZ,T_TH,T_PHI,T_PSI,ALTURAR,VELR,ALTURAF,VELF"},
          { LOG_ERR_MSG, sizeof(log_Errors),
          "3", "IfffffIfIfff", "T_MS,E_X,E_Y,E_Z,VOLT,AMP,SWT1,SWT2,GAS,ZDES,XDES,YDES"},
};

static void init_flash(){

     //************************** Data flash **************************************
        DataFlash.Init(log_structure, sizeof(log_structure)/sizeof(log_structure[0]));
         if (DataFlash.NeedErase()) {
                  DataFlash.EraseAll();
           }
        log_num = DataFlash.StartNewLog();
}


//Haciendo el guardado de datos

static void Log_Write_Pose(){

    struct log_Pose pkt = {
        LOG_PACKET_HEADER_INIT(LOG_POSE_MSG ),
        alt_barof    : alt_fil,
        Roll         : ahrs.roll,
        Pitch        : ahrs.pitch,
        Yaw          : ahrs.yaw,
        z_pos        : pos_z,
        x_vel        : vel_x,
        y_vel        : vel_y,
        z_vel        : vel_z,
        x_pos        : pos_x,
        y_pos        : pos_y,
        giroz        : gyroz,
        girox        : gyrox,
        giroy        : gyroy,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

static void Log_Write_Control(){
    struct log_Control pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_MSG),
        time_ms     : (float)(hal.scheduler->millis()/1000),
        u_z         : control_altura,
        tau_theta   : (c_x+ctrl.x),
        tau_phi     : (c_y+ctrl.y),
        tau_psi     : ctrl.z,
        alturaR     : altura,
        velR        : vel_alt,
        alturaF    : Xb1_d,
        velF    : Xb2_d,
    };

    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

static void Log_Write_Errors(){
    struct log_Errors pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ERR_MSG),
        time_ms        : (hal.scheduler->millis()/100),
        x_error        : error_x,
        y_error        : error_y,
        z_error        : error_z,
        voltaje        : volt,
        corriente      : corriente_tot,
        switch1         : aux_1,
        switch2        : aux_2,
        gas             : radio_throttle,
        z_des           :  ref_z,
        x_des           :  ref_x,
        y_des           :  ref_y,

    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
