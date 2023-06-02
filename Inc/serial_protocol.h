#ifndef __SERIAL_PROTOCOL_H
#define __SERIAL_PROTOCOL_H

#include "common.h"

enum DataLogTask
{
  DATALOG_TASK_FREE,
  DATALOG_TASK_START,
  DATALOG_TASK_SEND_DATA_SLOT_LEN,
  DATALOG_TASK_SEND_DATA_SLOT_MSG,
  DATALOG_TASK_DATALOG,
  DATALOG_TASK_END
};

typedef struct
{
  UART_HandleTypeDef*   huart;
  /* Receive */
  uint8_t               rxMsgRaw[255];
  uint8_t               rxMsgCfm[255];
  uint8_t               rxDataLen;//Max byte number = 255 - 6 = 249
  uint8_t               ifNewMsg;
  uint32_t              invalidRxMsgCount;
  /* Transmit */
  uint8_t               txMsg[255];
  uint8_t               txLen;
  /* Datalog */
  uint8_t               ifNewDatalogPiece2Send;
  uint8_t               ifDatalogInitialized;
  union UInt32UInt8     datalogIndex;
  uint8_t               dataSlotLen;
  uint32_t              datalogStartTimestamp;
  enum DataLogTask      datalogTask;
  uint8_t               datalogLabel2SendPtr;
}SerialProtocolHandle;

SerialProtocolHandle SERIALPROTOCOL_Create(UART_HandleTypeDef* huart);
void SERIALPROTOCOL_EnableCommunication(SerialProtocolHandle* hserial);
void SERIALPROTOCOL_SetNewDatalogSlotLength(SerialProtocolHandle* hserial, uint8_t len);
void SERIALPROTOCOL_TransmitCargo(SerialProtocolHandle* hserial, uint8_t* buf, uint8_t size);
void SERIALPROTOCOL_ReceiveCargoUARTIdleITCallback(SerialProtocolHandle* hserial);
void SERIALPROTOCOL_SendText(SerialProtocolHandle* hserial, char text[]);
void SERIALPROTOCOL_DatalogCargoReceiveManager(SerialProtocolHandle* hserial, void (*LabelSetFunc)(void));
void SERIALPROTOCOL_DatalogManager(SerialProtocolHandle* hserial, \
                                   void (*LabelSetFunc)(void), union FloatUInt8 dala_slots[]);
void SERIALPROTOCOL_DatalogSingleCargoTransmit(SerialProtocolHandle* hserial, union FloatUInt8 dala_slots[]);
void SERIALPROTOCOL_DatalogInitialization(SerialProtocolHandle* hserial);
void SERIALPROTOCOL_DatalogStart(SerialProtocolHandle* hserial);
void SERIALPROTOCOL_DatalogEnd(SerialProtocolHandle* hserial);
void SERIALPROTOCOL_SendDataSlotLen(SerialProtocolHandle* hserial);
void SERIALPROTOCOL_SendDataSlotLabel(SerialProtocolHandle* hserial, char* label_1, ...);
uint8_t SERIALPROTOCOL_CompareRxCfmMsgWithStr(SerialProtocolHandle* hserial, char str[], uint8_t size_of_str);
#endif
