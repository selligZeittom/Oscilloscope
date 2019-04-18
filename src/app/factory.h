#ifndef APP_FACTORY_H
#define APP_FACTORY_H

//
// What is seen only by the C++ compiler
//
#ifdef __cplusplus

#include "oscilloscopecontroller.h"
#include "gui.h"

#define ADC_VALUES_BUFFER_SIZE 1000

using oscilloscope::Gui;

/**
 * @brief Factory creating all objects/components and relations between them.
 */

//they are defined inside the isrs.cpp file
extern "C" uint16_t adcValuesBuffer[];
extern "C" uint32_t indexArray;

class Factory {
public:
	Factory();                          ///< Constructor

	static void initialize();           ///< Initializes the factory
	static void build();       ///< Creates components and initializes relations

	static OscilloscopeController & getOscilloscopeController();
	static Gui & getGui();

protected:
	static OscilloscopeController _oscilloscopeController;
	static Gui _gui;

};

#endif // __cplusplus

//
// What is seen by the C and C++ compiler
//
#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

void Factory_initialize();
void Factory_build();

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // APP_FACTORY_H
