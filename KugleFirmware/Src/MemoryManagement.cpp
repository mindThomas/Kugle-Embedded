#include "stm32h7xx_hal.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h" // for memset

#define USE_FREERTOS

extern int _sfreertos_heap; // get address to global symbol defined in linker file
extern uint8_t freeRTOSMemoryScheme;
extern "C" __EXPORT void ZeroInitFreeRTOSheap(void);

#if 1
/* Defining malloc/free should overwrite the
standard versions provided by the compiler. */
void * malloc (size_t size)
{
	/* Call the FreeRTOS version of malloc. */
	return pvPortMalloc( size );
}
void free (void * ptr)
{
	/* Call the FreeRTOS version of free. */
	vPortFree( ptr );
}
#endif

void ZeroInitFreeRTOSheap(void)
{
	uint8_t * heapPtr = (uint8_t*)&_sfreertos_heap; // heap is placed at this global symbol address
	freeRTOSMemoryScheme = configUSE_HEAP_SCHEME;
	memset(heapPtr, 0, configTOTAL_HEAP_SIZE);
}

#if 0
// Define the 'new' operator for C++ to use the freeRTOS memory management
// functions. THIS IS NOT OPTIONAL!
//
void * operator new(size_t size)
{
	void * p;

	#ifdef USE_FREERTOS
		if(uxTaskGetNumberOfTasks())
			p = pvPortMalloc(size);
		else
			p = malloc(size);

	#else
			p = malloc(size);
	#endif
	#ifdef __EXCEPTIONS
		if (p==0) // did pvPortMalloc succeed?
		throw std::bad_alloc(); // ANSI/ISO compliant behavior
	#endif
	return p;
}

//
// Define the 'delete' operator for C++ to use the freeRTOS memory management
// functions. THIS IS NOT OPTIONAL!
//
void operator delete(void *p)
{
	#ifdef USE_FREERTOS
		if(uxTaskGetNumberOfTasks())
			vPortFree( p );
		else
			free( p );
	#else
		free( p );
	#endif
	p = NULL;
}
#endif

#if 0
/* Optionally you can override the 'nothrow' versions as well.
This is useful if you want to catch failed allocs with your
own debug code, or keep track of heap usage for example,
rather than just eliminate exceptions.
*/

void* operator new(std::size_t size, const std::nothrow_t&) {
return malloc(size);
}

void* operator new {
return malloc(size);
}

void operator delete(void* ptr, const std::nothrow_t&) {
free(ptr);
}

void operator delete {
free(ptr);
}

#endif
