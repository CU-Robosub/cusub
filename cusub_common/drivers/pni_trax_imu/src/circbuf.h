#ifndef CIRCBUF_H
#define CIRCBUF_H
#include <stdint.h>

/*********************************************************************
*CB_Enum_t
*	description:Is the typedef enum that deals with all the possible
				cases of failure/success in the circular buffer that
				is being used. All circular buffer related functions
				have CB_Enum_t as the return type.
*	parameters:N/A

*	returns:N/A
*********************************************************************/

typedef enum CBe
{
	NO_ERROR = 0,
	BUFFER_EMPTY,
	BUFFER_FULL,
	NULL_POINTER,
	NO_LENGTH,
	BUFFER_ALLOCATION_FAILED,
}CB_Enum_t;

/*********************************************************************
*CircBuf_t
*	description:Is the typedef struct datatype that is related to each
				circular buffer that is created. The elements of this 
				data structure are:
					1. Array in which data is stored 
					2. Pointer buffer, points to first element
					3. Pointer head, tracks the current head position
					4. Pointer tail, tracks the current tail position
					5. size_t num_items, tracks current numnber of 
					   elements in the array
					6. size_t item_size, the size of each individual 
					   item in the buffer
					7. size_t buf_size, the size of the buffer

*	parameters:N/A

*	returns:N/A
*********************************************************************/

typedef struct CircBuf
{
	uint8_t * buffer;
	uint8_t * head;
	uint8_t * tail;
	uint8_t send_status;
	size_t size;
	size_t num_items;
}CircBuf_t;

/*List of function prototypes*/


/*********************************************************************
*circbuf_init
*	description:Initializes the buffer to the size as specified by the
				arguments that are passed into this initialization fun
				-ction. 
*	parameters:
		A pointer to the buffer is passed
		size_t size_of_buffer - The maximum number of elements
*	returns:
			Returns the success or failure based on the CB_Enum_t retu
			-rn type
*********************************************************************/

CB_Enum_t circbuf_init(CircBuf_t *circ, size_t size_of_buffer);

/*********************************************************************
*circbuf_add
*	description:Adds an element to the circular buffer 
*	parameters:
		A pointer to the buffer is passed
		data of type uint8_t is passed as an argument
*	returns:
			Returns the success or failure based on the CB_Enum_t retu
			-rn type
*********************************************************************/

CB_Enum_t circbuf_add(CircBuf_t *circ, uint8_t data);

/*********************************************************************
*circbuf_remove
*	description:Removes an element to the circular buffer 
*	parameters:
		A pointer to the buffer is passed
		Variable to store the data removed
*	returns:
			Returns the success or failure based on the CB_Enum_t retu
			-rn type
*********************************************************************/

CB_Enum_t circbuf_remove(CircBuf_t *circ, uint8_t *data_ptr);

/*********************************************************************
*circbuf_full
*	description:Checks if the buffer is full or not full
*	parameters:
		A pointer to the buffer is passed
		Circular buffer type object to be checked
*	returns:
			Returns the success or failure based on the CB_Enum_t retu
			-rn type
*********************************************************************/

CB_Enum_t circbuf_full(CircBuf_t *circ);

/*********************************************************************
*circbuf_empty
*	description:Checks if the buffer is empty or not empty
*	parameters:
		Circular buffer type object to be checked
*	returns:
			Returns the success or failure based on the CB_Enum_t retu
			-rn type
*********************************************************************/

CB_Enum_t circbuf_empty(CircBuf_t *circ);

/*********************************************************************
*circbuf_peek
*	description:Reads the nth element of the circular buffer 
*	parameters:
		A pointer to the buffer is passed
		Index of element to be seen
		Variable to store the data removed
*	returns:
			Returns the success or failure based on the CB_Enum_t retu
			-rn type
*********************************************************************/

CB_Enum_t circbuf_peek(CircBuf_t *circ, size_t index, uint8_t *data_element);

/*********************************************************************
*circbuf_destroy
*	description:Frees the circular buffer from memory 
*	parameters:
		Circular buffer type object to be deleted
*	returns:
			Returns the success or failure based on the CB_Enum_t retu
			-rn type
*********************************************************************/

CB_Enum_t circbuf_destroy(CircBuf_t *circ);

#endif
