#include "serial_protocol.h"

SerialProtocolHandle SERIALPROTOCOL_Create(UART_HandleTypeDef* huart)
{
  SerialProtocolHandle hserial;
  hserial.huart = huart;
  hserial.ifNewMsg = 0;
  hserial.invalidRxMsgCount = 0;
  
  hserial.ifNewDatalogPiece2Send = 0;
  hserial.ifDatalogInitialized = 0;
  hserial.datalogIndex.b32 = 0;
  hserial.dataSlotLen = 0;
  hserial.datalogStartTimestamp = 0;
  hserial.datalogTask = DATALOG_TASK_FREE;
  
  hserial.txMsg[0] = 0xAA;
  hserial.txMsg[1] = 0xCC;
  return hserial;
}

void SERIALPROTOCOL_SetNewDatalogSlotLength(SerialProtocolHandle* hserial, uint8_t len)
{
  hserial->dataSlotLen = len;
}

void SERIALPROTOCOL_SetNewDatalogSlot(SerialProtocolHandle* hserial, union FloatUInt8 (*data_slot))
{
  hserial->dataSlot = data_slot;
}

void SERIALPROTOCOL_SetNewDatalogSendLabelFunction(SerialProtocolHandle* hserial, FuncTypeVoidVoid func)
{
  hserial->SetLabelFunc = func;
}

void SERIALPROTOCOL_TransmitCargo(SerialProtocolHandle* hserial, uint8_t* buf, uint8_t size)
{
  hserial->txLen = size + 6;
  hserial->txMsg[2] = size;
  memcpy(&hserial->txMsg[3], buf, size);
  
  union UInt16UInt8 crc;
  crc.b16 = CRC16_Modbus(hserial->txMsg, size + 3);
  
  hserial->txMsg[size + 3] = crc.b8[0];
  hserial->txMsg[size + 4] = crc.b8[1];
  hserial->txMsg[size + 5] = 0x55;
  HAL_UART_Transmit_DMA(hserial->huart, hserial->txMsg, hserial->txLen);
}

void SERIALPROTOCOL_ReceiveCargoUARTIdleITCallback(SerialProtocolHandle* hserial)
{
  HAL_UART_DMAStop(hserial->huart);//Restart the DMA receiver
  uint8_t i = 0;
//  hserial->dmaPtr  = __HAL_DMA_GET_COUNTER(hserial->huart->hdmarx);
  while (i != 255)//Pointer for scanning through the buffer
  {
    if (hserial->rxMsgRaw[i] == 0xAA && hserial->rxMsgRaw[i + 1] == 0xCC)//Find the start delimiter
    {
      uint8_t tem_size = hserial->rxMsgRaw[i + 2];
      if (tem_size + i > 250)//Msg out of RX buffer range
      {
        hserial->invalidRxMsgCount++;
        break;
      }
      if (hserial->rxMsgRaw[tem_size + 5 + i] == 0x55)//Find the end delimiter
      {
        hserial->rxDataLen = tem_size;
        union UInt16UInt8 temCRC;
        temCRC.b16 = CRC16_Modbus(hserial->rxMsgRaw + i, tem_size + 3);
        if(1)//if (temCRC.b8[0] == hserial->rxMsgRaw[tem_size + 3 + i] && temCRC.b8[1] == hserial->rxMsgRaw[tem_size + 4 + i])//CRC16 modebus check
        {
          hserial->ifNewMsg = 1;
          memcpy(hserial->rxMsgCfm, hserial->rxMsgRaw + 3 + i, hserial->rxDataLen);
          break;
        }
        else
          hserial->invalidRxMsgCount++;
      }
    }
    i++;
  }
  HAL_UARTEx_ReceiveToIdle_DMA(hserial->huart, hserial->rxMsgRaw, 255);
}

void SERIALPROTOCOL_EnableCommunication(SerialProtocolHandle* hserial)
{
  HAL_UARTEx_ReceiveToIdle_DMA(hserial->huart, hserial->rxMsgRaw, 255);
}

void SERIALPROTOCOL_SendText(SerialProtocolHandle* hserial, char text[])
{
  SERIALPROTOCOL_TransmitCargo(hserial, (uint8_t*)text, strlen(text));
}

void SERIALPROTOCOL_DatalogCargoReceiveManager(SerialProtocolHandle* hserial)
{
  ////////////////////*Force to the following states*//////////
  if(SERIALPROTOCOL_IfNewMsgAndItIsTheString(hserial, "Datalog start", 0))
  {
    SERIALPROTOCOL_DatalogPassivelyStart(hserial);
    return;
  }
  else if(SERIALPROTOCOL_IfNewMsgAndItIsTheString(hserial, "Datalog end", 0))
    hserial->datalogTask = DATALOG_TASK_END_PASSIVE;
  ///////////////////* Sequencer as the following states*//////
  if (hserial->datalogTask == DATALOG_TASK_FREE){}
  else if (hserial->datalogTask == DATALOG_TASK_START_ACTIVE)
  {
    if(SERIALPROTOCOL_IfNewMsgAndItIsTheString(hserial, "Datalog start request confirmed!", 0))
      hserial->datalogTask = DATALOG_TASK_SEND_DATA_SLOT_LEN;
  }
  else if (hserial->datalogTask == DATALOG_TASK_SEND_DATA_SLOT_LEN)
  {
    if(SERIALPROTOCOL_IfNewMsgAndItIsTheString(hserial, "Data slot length received!", 0))
      hserial->datalogTask = DATALOG_TASK_START_SEND_DATA_SLOT_LABEL;
  }
  else if (hserial->datalogTask == DATALOG_TASK_START_SEND_DATA_SLOT_LABEL)
  {
    if (hserial->rxDataLen == 1)
    {
      hserial->datalogTask = DATALOG_TASK_SEND_DATA_SLOT_LABEL;
      hserial->datalogLabel2SendPtr = 0;
      hserial->ifNewMsg = 0;
    }
  }
  else if (hserial->datalogTask == DATALOG_TASK_SEND_DATA_SLOT_LABEL)
  {
    if (hserial->ifNewMsg)
    {
      if (hserial->rxDataLen == 1)
        hserial->datalogLabel2SendPtr = hserial->rxMsgCfm[0] - 1;
      if (hserial->rxMsgCfm[0] == (hserial->dataSlotLen + 1))
        hserial->datalogTask = DATALOG_TASK_DATALOG;
      hserial->ifNewMsg = 0;
    }
  }
  else if (hserial->datalogTask == DATALOG_TASK_DATALOG){}
  else if (hserial->datalogTask == DATALOG_TASK_END_ACTIVE)
  {
    if(SERIALPROTOCOL_IfNewMsgAndItIsTheString(hserial, "Roger that", 0))
      hserial->datalogTask = DATALOG_TASK_FREE;
  }
  else if (hserial->datalogTask == DATALOG_TASK_END_PASSIVE)
  {
    if(SERIALPROTOCOL_IfNewMsgAndItIsTheString(hserial, "Datalog free", 0))
      hserial->datalogTask = DATALOG_TASK_FREE;
  }
}

void SERIALPROTOCOL_DatalogCargoTransmitManager(SerialProtocolHandle* hserial, void (*LabelSetFunc)(void), union FloatUInt8 data_slots[])
{
  if (hserial->datalogTask == DATALOG_TASK_START_ACTIVE)
    SERIALPROTOCOL_SendText(hserial, "Datalog start");
  else if (hserial->datalogTask == DATALOG_TASK_START_PASSIVE){}
  else if (hserial->datalogTask == DATALOG_TASK_SEND_DATA_SLOT_LEN)
    SERIALPROTOCOL_SendDataSlotLen(hserial);
  else if (hserial->datalogTask == DATALOG_TASK_START_SEND_DATA_SLOT_LABEL)
    SERIALPROTOCOL_SendText(hserial, "Datalog label");
  else if (hserial->datalogTask == DATALOG_TASK_SEND_DATA_SLOT_LABEL)
    (*LabelSetFunc)();
  else if (hserial->datalogTask == DATALOG_TASK_DATALOG)
  {
    if (hserial->ifNewDatalogPiece2Send)
    {
      SERIALPROTOCOL_DatalogSingleCargoTransmit(hserial, data_slots);
      hserial->ifNewDatalogPiece2Send = 0;
    }
  }
  else if (hserial->datalogTask == DATALOG_TASK_END_ACTIVE)
    SERIALPROTOCOL_SendText(hserial, "Datalog end");
  else if (hserial->datalogTask == DATALOG_TASK_END_PASSIVE)
    SERIALPROTOCOL_SendText(hserial, "Datalog Ready to End!");
}

void SERIALPROTOCOL_DatalogManager(SerialProtocolHandle* hserial)
{
  SERIALPROTOCOL_DatalogCargoReceiveManager(hserial);
  SERIALPROTOCOL_DatalogCargoTransmitManager(hserial, hserial->SetLabelFunc, hserial->dataSlot);
}

void SERIALPROTOCOL_DatalogSingleCargoTransmit(SerialProtocolHandle* hserial, union FloatUInt8 data_slots[])
{
  uint8_t i = 0;
  union UInt32UInt8 sysTick;
  sysTick.b32 = HAL_GetTick();
  //Index
  hserial->datalogBuf[i++] = hserial->datalogIndex.b8[0];
  hserial->datalogBuf[i++] = hserial->datalogIndex.b8[1];
  hserial->datalogBuf[i++] = hserial->datalogIndex.b8[2];
  hserial->datalogBuf[i++] = hserial->datalogIndex.b8[3];
  hserial->datalogIndex.b32++;
  //Time stamp
  hserial->datalogBuf[i++] = sysTick.b8[0];
  hserial->datalogBuf[i++] = sysTick.b8[1];
  hserial->datalogBuf[i++] = sysTick.b8[2];
  hserial->datalogBuf[i++] = sysTick.b8[3];
  //Data
  for (uint8_t j = 0; j < hserial->dataSlotLen; j++)
  {
    hserial->datalogBuf[i++] = data_slots[j].b8[0];
    hserial->datalogBuf[i++] = data_slots[j].b8[1];
    hserial->datalogBuf[i++] = data_slots[j].b8[2];
    hserial->datalogBuf[i++] = data_slots[j].b8[3];
  }
  SERIALPROTOCOL_TransmitCargo(hserial, hserial->datalogBuf, hserial->dataSlotLen * 4 + 8);
}

void SERIALPROTOCOL_DatalogInitialization(SerialProtocolHandle* hserial)
{
  hserial->ifDatalogInitialized = 1;
  hserial->ifNewDatalogPiece2Send = 0;
  hserial->datalogIndex.b32 = 0;
}

void SERIALPROTOCOL_DatalogInitiateStart(SerialProtocolHandle* hserial)
{
  SERIALPROTOCOL_DatalogInitialization(hserial);
  hserial->datalogTask = DATALOG_TASK_START_ACTIVE;
}

void SERIALPROTOCOL_DatalogPassivelyStart(SerialProtocolHandle* hserial)
{
  SERIALPROTOCOL_DatalogInitialization(hserial);
  hserial->datalogTask = DATALOG_TASK_SEND_DATA_SLOT_LEN;
}

void SERIALPROTOCOL_DatalogInitiateEnd(SerialProtocolHandle* hserial)
{
  hserial->datalogTask = DATALOG_TASK_END_ACTIVE;
  SERIALPROTOCOL_SendText(hserial, "Datalog end");
}

void SERIALPROTOCOL_SendDataSlotLen(SerialProtocolHandle* hserial)
{
  char numStr[2];
  char txStr[7];
  strncpy(txStr, "len: ", 5);
  int numStrLen;
  numStrLen = sprintf(numStr, "%d", hserial->dataSlotLen);
  strncpy(&txStr[5], numStr, numStrLen);
  SERIALPROTOCOL_TransmitCargo(hserial, (uint8_t*)txStr, numStrLen + 5);
}

void SERIALPROTOCOL_SendDataSlotLabel(SerialProtocolHandle* hserial, char* label_1, ...)
{
  va_list label_ptr;
  va_start(label_ptr, label_1);
 
  uint8_t numOfLabels = atoi(label_1);
  uint32_t  startSendingTimeStamp = HAL_GetTick();
  for (uint8_t i = 0; i < numOfLabels; i++)
  {
    char buf[50];
    buf[0] = i + 1;
    strcpy(&buf[1], va_arg(label_ptr, char*));
    if (i == hserial->datalogLabel2SendPtr)
    {
      SERIALPROTOCOL_TransmitCargo(hserial, (uint8_t*)buf, strlen(buf));
      break;
    }
  }
  va_end(label_ptr);
}

uint8_t SERIALPROTOCOL_IfNewMsgAndItIsTheString(SerialProtocolHandle* hserial, char str[], uint8_t pos)
{
  if (hserial->ifNewMsg)
  {
    if (!strncmp((const char*)(hserial->rxMsgCfm + pos), (const char*)str, strlen(str)))
    {
      hserial->ifNewMsg = 0;
      return 1;
    }
  }
  return 0;
}
