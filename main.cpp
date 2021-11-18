#include "mbed.h"
#include "wifi.h" ///////////////////// AGREGAR ///////////////////////////////////


#define NUMBEAT             4

#define HEARBEATINTERVAL    100

#define RINGBUFFLENGTH      256



/**
 * @brief Enumeración de la MEF para decodificar el protocolo
 * 
 */
typedef enum{
    START,
    HEADER_1,
    HEADER_2,
    HEADER_3,
    NBYTES,
    TOKEN,
    PAYLOAD
}_eProtocolo;

_eProtocolo estadoProtocolo;

/**
 * @brief Enumeración de la lista de comandos
 * 
 */
 typedef enum{
        ACK=0x0D,
        ALIVE=0xF0,
        STARTCONFIG=0xEE,
        OTHERS
    }_eID;

wifiData myWifiData; ///////////////////// AGREGAR ///////////////////////////////////

/**
 * @brief Estructura de datos para el puerto serie
 * 
 */
typedef struct{
    uint8_t timeOut;         //!< TiemOut para reiniciar la máquina si se interrumpe la comunicación
    uint8_t indexStart; ///////////////////// AGREGAR ///////////////////////////////////
    uint8_t cheksumRx;       //!< Cheksumm RX
    uint8_t indexWriteRx;    //!< Indice de escritura del buffer circular de recepción
    uint8_t indexReadRx;     //!< Indice de lectura del buffer circular de recepción
    uint8_t indexWriteTx;    //!< Indice de escritura del buffer circular de transmisión
    uint8_t indexReadTx;     //!< Indice de lectura del buffer circular de transmisión
    uint8_t bufferRx[256];   //!< Buffer circular de recepción
    uint8_t bufferTx[256];   //!< Buffer circular de transmisión
}_sDato ;

 _sDato datosComProtocol, datosComWifi;

uint8_t hearBeatEvent;

/**
 * @brief Unión para descomponer/componer datos mayores a 1 byte
 * 
 */
typedef union {
    int32_t i32;
    uint32_t ui32;
    uint16_t ui16[2];
    uint8_t ui8[4];
}_udat;

_udat myWord;


/*************************************************************************************************/
/* Prototipo de Funciones */


/**
 * @brief Función que se llama en la interrupción de recepción de datos
 * Cuando se llama la fucnión se leen todos los datos que llagaron.
 */
void onDataRx(void);

/**
 * @brief Decodifica las tramas que se reciben 
 * La función decodifica el protocolo para saber si lo que llegó es válido.
 * Utiliza una máquina de estado para decodificar el paquete
 */
void decodeProtocol(_sDato *);

/**
 * @brief Procesa el comando (ID) que se recibió
 * Si el protocolo es correcto, se llama a esta función para procesar el comando
 */
void decodeData(_sDato *);

/**
 * @brief Envía los datos a la PC puerto Serie
 * La función consulta si el puerto serie está libra para escribir, si es así envía 1 byte y retorna
 */
void sendData();

/**
 * @brief Envía los datos a la PC por WIFI
 * 
 */
void sendDataWifi();


/**
 * @brief Función Hearbeat
 * Ejecuta las tareas del hearbeat 
 */
void hearbeatTask(void);


/*****************************************************************************************************/
/* Configuración del Microcontrolador */

DigitalOut HEARBEAT(PC_13); //!< Defino la salida del led

Serial pcCom(PA_9,PA_10,115200); //!< Configuración del puerto serie, la velocidad (115200) tiene que ser la misma en QT

Timer miTimer; //!< Timer general

/**
 * @brief Clase Wifi utilizada para configurar el ESP8266_01 y manejar la comunicación
 * con el mismo 
 */
Wifi myWifi(datosComWifi.bufferRx,&datosComWifi.indexWriteRx, sizeof(datosComWifi.bufferRx));/////////////////// AGREGAR ///////////////////////////////////

/*****************************************************************************************************/
/************  Función Principal ***********************/


int main()
{
    int hearbeatTime=0;

    miTimer.start();

    hearBeatEvent=0;
    
    pcCom.attach(&onDataRx,Serial::RxIrq);

    myWifi.initTask();///////////////////// AGREGAR ///////////////////////////////////
    
    while(true)
    {

        myWifi.taskWifi(); ///////////////////// AGREGAR ///////////////////////////////////
     
        if ((miTimer.read_ms()-hearbeatTime)>=HEARBEATINTERVAL){
            hearbeatTime=miTimer.read_ms();
            hearbeatTask();
        }
        //****************** puerto serie ***********************
        if(datosComProtocol.indexReadRx!=datosComProtocol.indexWriteRx){ 
            decodeProtocol(&datosComProtocol);
        }

        if(datosComProtocol.indexReadTx!=datosComProtocol.indexWriteTx){
            sendData();
        } 
 
       //********************** wifi ************************
        if(datosComWifi.indexReadRx!=datosComWifi.indexWriteRx){
            decodeProtocol(&datosComWifi);
        }

        if(datosComWifi.indexReadTx!=datosComWifi.indexWriteTx){
            sendDataWifi();
        }

        ///////////////////  OTRA ALTERNATIVA ////////////////////////////
        /*
        if(datosComWifi.indexReadTx!=datosComWifi.indexWriteTx){
            myWifi.writeWifiData(&datosComWifi.bufferTx[datosComWifi.indexReadTx++],1);
        }
        */
        
    }
    return 0;
}





/*****************************************************************************************************/
/************  MEF para decodificar el protocolo serie ***********************/
void decodeProtocol(_sDato *datosCom)
{
    static uint8_t nBytes=0; ///////////////////// TIENE QUE SER UNSIGNED ///////////////////////////////////
    while (datosCom->indexReadRx!=datosCom->indexWriteRx)
    {
        switch (estadoProtocolo) {
            case START:
                if (datosCom->bufferRx[datosCom->indexReadRx++]=='U'){
                    estadoProtocolo=HEADER_1;
                    datosCom->cheksumRx=0;
                }
                break;
            case HEADER_1:
                if (datosCom->bufferRx[datosCom->indexReadRx++]=='N')
                   estadoProtocolo=HEADER_2;
                else{
                    datosCom->indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
            case HEADER_2:
                if (datosCom->bufferRx[datosCom->indexReadRx++]=='E')
                    estadoProtocolo=HEADER_3;
                else{
                    datosCom->indexReadRx--;
                   estadoProtocolo=START;
                }
                break;
        case HEADER_3:
            if (datosCom->bufferRx[datosCom->indexReadRx++]=='R')
                estadoProtocolo=NBYTES;
            else{
                datosCom->indexReadRx--;
               estadoProtocolo=START;
            }
            break;
            case NBYTES:
                datosCom->indexStart=datosCom->indexReadRx; ///////////////////// AGREGAR ///////////////////////////////////
                nBytes=datosCom->bufferRx[datosCom->indexReadRx++];
                estadoProtocolo=TOKEN;
                break;
            case TOKEN:
                if (datosCom->bufferRx[datosCom->indexReadRx++]==':'){
                   estadoProtocolo=PAYLOAD;
                    datosCom->cheksumRx ='U'^'N'^'E'^'R'^ nBytes^':';
                }
                else{
                    datosCom->indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
            case PAYLOAD:
                if (nBytes>1){
                    datosCom->cheksumRx ^= datosCom->bufferRx[datosCom->indexReadRx++];
                }
                nBytes--;
                if(nBytes<=0){
                   
                    estadoProtocolo=START;
                    if(datosCom->cheksumRx == datosCom->bufferRx[datosCom->indexReadRx]){
                        decodeData(datosCom); 
                    }
                }
               
                break;
            default:
                estadoProtocolo=START;
                break;
        }
    }
    

}


/*****************************************************************************************************/
/************  Función para procesar el comando recibido ***********************/
void decodeData(_sDato *datosCom)
{
    #define POSID   2
    #define POSDATA 3
    wifiData *wifidataPtr;///////////////////// AGREGAR ///////////////////////////////////
    uint8_t *ptr; ///////////////////// AGREGAR ///////////////////////////////////
    uint8_t auxBuffTx[50], indiceAux=0, cheksum, sizeWifiData, indexBytesToCopy=0, numBytesToCopy=0;
    auxBuffTx[indiceAux++]='U';
    auxBuffTx[indiceAux++]='N';
    auxBuffTx[indiceAux++]='E';
    auxBuffTx[indiceAux++]='R';
    auxBuffTx[indiceAux++]=0;
    auxBuffTx[indiceAux++]=':';

    switch (datosCom->bufferRx[datosCom->indexStart+POSID]) {
        case ALIVE:
            auxBuffTx[indiceAux++]=ALIVE;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x03;         
            break;
            ///////////////////// AGREGAR INICIO///////////////////////////////////
        case STARTCONFIG: //Inicia Configuración del wifi 
            sizeWifiData =sizeof(myWifiData);
            indexBytesToCopy=datosCom->indexStart+POSDATA;
            wifidataPtr=&myWifiData;

            if ((RINGBUFFLENGTH - indexBytesToCopy)<sizeWifiData){
                numBytesToCopy=RINGBUFFLENGTH-indexBytesToCopy;
                memcpy(wifidataPtr,&datosCom->bufferRx[indexBytesToCopy], numBytesToCopy);
                indexBytesToCopy+=numBytesToCopy;
                sizeWifiData-=numBytesToCopy;
                ptr= (uint8_t *)wifidataPtr + numBytesToCopy;
                memcpy(ptr,&datosCom->bufferRx[indexBytesToCopy], sizeWifiData);
            }else{
                memcpy(&myWifiData,&datosCom->bufferRx[indexBytesToCopy], sizeWifiData);
            }
            myWifi.configWifi(&myWifiData);
            ///////////////////// AGREGAR FIN ///////////////////////////////////
            break;
        default:
            auxBuffTx[indiceAux++]=0xDD; // NO COMPRENDÍ EL ID //
            auxBuffTx[NBYTES]=0x02;
            break;
    }
   cheksum=0;
    for(uint8_t a=0 ;a < indiceAux ;a++)
    {
        cheksum ^= auxBuffTx[a];
        datosCom->bufferTx[datosCom->indexWriteTx++]=auxBuffTx[a];
    }
        datosCom->bufferTx[datosCom->indexWriteTx++]=cheksum;
  

}


/*****************************************************************************************************/
/************  Función para enviar los bytes hacia la pc ***********************/
void sendData()
{
    if(pcCom.writable())
        pcCom.putc(datosComProtocol.bufferTx[datosComProtocol.indexReadTx++]);

}

///////////////////// AGREGAR  INICIO ///////////////////////////////////
void sendDataWifi()
{
    uint8_t numbytes;
    uint8_t *buff;
    
    if(datosComWifi.indexReadTx > datosComWifi.indexWriteTx)
        numbytes= datosComWifi.indexWriteTx - datosComWifi.indexReadTx + 0xFF;
    else
        numbytes= datosComWifi.indexWriteTx -datosComWifi.indexReadTx;

    buff = (uint8_t*)malloc(numbytes);
    
    if(buff!=NULL){
        for(uint8_t i=0; i<numbytes; i++)
            buff[i]= datosComWifi.bufferTx[datosComWifi.indexReadTx++];
        
        myWifi.writeWifiData(buff,numbytes);
        
        free(buff);
    }
}
///////////////////// AGREGAR  FIN ///////////////////////////////////

/*****************************************************************************************************/
/************  Función para hacer el hearbeats ***********************/
void hearbeatTask(void)
{
    if(hearBeatEvent < NUMBEAT){
        HEARBEAT=!HEARBEAT;
        hearBeatEvent++;
    }else{
        HEARBEAT=1;
        hearBeatEvent = (hearBeatEvent>=25) ? (0) : (hearBeatEvent+1);    
    }
}


/**********************************************************************************/
/* Servicio de Interrupciones*/

void onDataRx(void)
{
    while (pcCom.readable())
    {
        datosComProtocol.bufferRx[datosComProtocol.indexWriteRx++]=pcCom.getc();
    }
}


