#include "xml.h"

uint8_t SDCARD_XML_FindTag(SDCardHandle* hsdcard, uint32_t search_start_ptr, uint32_t search_end_ptr, uint32_t* tag_start, uint32_t* tag_end, char* file_name, char* tag_name, uint8_t sizeof_tag_name)
{
  if (f_open(&hsdcard->SDFile, file_name, FA_READ) != FR_OK)
    return 1;
  f_lseek(&hsdcard->SDFile, search_start_ptr);
  
  uint32_t read_number;
  read_number = 1;
  do{
    f_read(&hSDCard.SDFile, &read_byte, read_number, (UINT *)&read_number);
  }while(read_byte != '<');
  read_number = sizeof_tag_name;
  f_read(&hSDCard.SDFile, read_tag, read_number, (UINT *)&read_number);
  
  
  return 0;
}
