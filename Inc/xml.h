#ifndef XML_H
#define XML_H

uint8_t SDCARD_XML_FindTag(SDCardHandle* hsdcard, uint32_t search_start_ptr, uint32_t search_end_ptr, uint32_t* tag_start, uint32_t* tag_end, char* file_name, char* tag_name, uint8_t sizeof_tag_name);





#endif
