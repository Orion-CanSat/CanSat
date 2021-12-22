#include <usb_names.h>

#define MANUFACTURER_NAME { 'O', 'R', 'I', 'O', 'N', '-', 'E', 'L', 'E', 'C', 'T', 'R', 'O', 'N', 'I', 'C', 'S', '-', 'A', 'N', 'D', '-', 'S', 'O', 'F', 'T', 'W', 'A', 'R', 'E' }
#define MANUFACTURER_NAME_LEN 30

#define PRODUCT_NAME { 'B', 'E', 'T', 'E', 'L', 'G', 'E', 'U', 'S', 'E' }
#define PRODUCT_NAME_LEN 10

#define SERIAL_NUMBER { 'B', 'E', 'T', 'E', 'L', 'G', 'E', 'U', 'S', 'E', '-', '0', '0', '1' }
#define SERIAL_NUMBER_LEN 14

struct usb_string_descriptor_struct usb_string_manufacturer_name = {
    2 + MANUFACTURER_NAME_LEN * 2,
    3,
    MANUFACTURER_NAME};

struct usb_string_descriptor_struct usb_string_product_name = {
    2 + PRODUCT_NAME_LEN * 2,
    3,
    PRODUCT_NAME};

struct usb_string_descriptor_struct usb_string_serial_number = {
    2 + SERIAL_NUMBER_LEN * 2,
    3,
    SERIAL_NUMBER};