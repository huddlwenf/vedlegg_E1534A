

/* This app will do squearwave frequence measurements and advertise the result over bluetooth.
	 The advertising will be executed every five secons, 
	 and frequency measurements will be done with a one hour intervall.
   Radionotification is used to enter a radio notification handler after radio activity.
	 This handler will count up to where the measurement will take place. 5sec * 720 = 3600sec.
	 APP_TIMER is set up with a timeout of one second, 
	 and GPIOTE with event triggered on positive edge sensing will do a count task inn TIMER2 via a PPI channel.
	 The timeouthandler will do a readout of the TIMER2 countvalue, and update the advertising data.
   Pin 0.15 senses frequency, and pin 0.16 is used for powering the 555-timer.
*/     
 
#include <stdbool.h>
#include <stdint.h>
#include "ble_advdata.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_gpio.h"
#include "ble_radio_notification.h" 
#include "app_gpiote.h"
#include "nrf_ppi.h"


#define IS_SRVC_CHANGED_CHARACT_PRESENT 1                                             /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/
 
#define APP_CFG_NON_CONN_ADV_TIMEOUT    0                                             /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(5000, UNIT_0_625_MS)            /**< The advertising interval for non-connectable advertisement (5 s). This value can vary between 100ms to 10.24s). */
#define TX_POWER                        0                                             /**< TX Power in dBm. */

#define APP_TIMER_PRESCALER             0                                             /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            (3+BSP_APP_TIMERS_NUMBER)                     /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         5                                             /**< Size of timer operation queues. */
#define FREQ_MEAS_TIME                  APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)
#define MEASURE_INTERVAL                720                                           // MEASURE_INTERVAL * 5 sec = How ofte it takes a measurement in seconds.
                                                                                      // MEASURE_INTERVAL havea minimum value of '2'.



uint32_t advcount = 0;                                                                // Variable for counting advertisements.
static app_timer_id_t         m_freq_meas_timer_id;                                   // Name for the APP_TIMER used for measurement timeout.
  
static ble_gap_adv_params_t   m_adv_params;                                           /**< Parameters to be passed to the stack when starting advertising. */

ble_advdata_t                 m_advdata;
uint8_t                       m_servicedata_count = 1; 
ble_advdata_service_data_t    m_service_data;
uint16_t                      m_uuid = 0x1809;
uint8_array_t                 m_service_data_array;
uint8_t                       m_service_data_data[1];
uint16_t                      m_size = sizeof(m_service_data_data);

uint8_t                       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;








/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init()
{	
	  m_service_data_data[0] = 0xab;
	  //m_service_data_data[1] = 0xcd;
	
	  m_service_data_array.p_data         = m_service_data_data;
	  m_service_data_array.size           = m_size;
	  m_service_data.service_uuid         = m_uuid;
	  m_service_data.data                 = m_service_data_array; 
	  
	
    memset(&m_advdata, 0, sizeof(m_advdata));
   
	  m_advdata.flags                     = true;  
    m_advdata.name_type                 = BLE_ADVDATA_NO_NAME;
    m_advdata.flags                     = flags;
    
		m_advdata.p_service_data_array      = &m_service_data;
    m_advdata.service_data_count        = m_servicedata_count;
		
		ble_advdata_set(&m_advdata, NULL);
    

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type                 = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr          = NULL;                               // Undirected advertisement.
    m_adv_params.fp                   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval             = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout              = APP_CFG_NON_CONN_ADV_TIMEOUT;
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{ 
    sd_ble_gap_adv_start(&m_adv_params);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    sd_ble_enable(&ble_enable_params);
}


/**@brief Function for doing power management.
 */
static void power_manage(void)
{ 
	  sd_app_evt_wait();                                                      // Sleep mode.
}  


void gpio_cnf(void)
{ 
	  nrf_gpio_pin_dir_set(15, 0);                                            // pin 15 input. Used for sensing frequency from 555-timer.  
	  nrf_gpio_pin_dir_set(16, 1);                                            // pin 16 output. Used for driving 555-timer.
	  nrf_gpio_pin_dir_set(03, 1);                                            // pin 03 output, Used for disable ADP5090 when radio is acrive.
	  nrf_gpio_pin_clear(3);
    nrf_gpio_pin_set(16);	                                                  
}




void GPIOTE_cnf(void)
{   
	  NRF_GPIOTE->CONFIG[0] = 0x10f01;                                        //Event on rising edge on pin 0.15. 	
}



void timer2_cnf(void)
{
    NRF_TIMER2->MODE    = 0x01;                                             // Counter mode.
  	NRF_TIMER2->BITMODE = 0;                                                // 16-bit.
}


void ppi_cnf(void)
{ 
	  NRF_PPI->CHEN      |= 0x01;                                             // Enable PPI-channel 0.
	  
	  /*PPI channel 0 will do a count task in TIMER 2
     when an event is registered on pin 0.15.         */ 	
    NRF_PPI->CH[0].EEP  = (uint32_t)&NRF_GPIOTE->EVENTS_IN[0]; 
	  NRF_PPI->CH[0].TEP  = (uint32_t)&NRF_TIMER2->TASKS_COUNT;	
}







void radio_notif_handler_evt(bool radio_active)
{
	    
	if(radio_active==true)
	{                                                                         // If just before radio start sending packet.
		sd_power_dcdc_mode_set(1);                                              // Enable DCDC-refulator. 
		nrf_gpio_pin_set(3);			                                              // Disable ADP5090.  
	}
	
	
	
	if(radio_active==false)                                                   // If just after radio have sent packet.
	{      
		advcount++;                                                             // Counter for meassure interval.
		sd_power_dcdc_mode_set(0);                                              // Disable DCDC-regulator.
		nrf_gpio_pin_clear(3);                                                  // Enable ADP5090.
				
		
				  
			//First measurement.
			if(m_service_data_data[0]==0xab)                 
			{ 
		 		NRF_GPIOTE->CONFIG[0] = 0x10f01;                                    //Event on rising edge on pin15.
				nrf_gpio_pin_set(16);                                               // Turn on 555 timer.
				NRF_TIMER2->TASKS_CLEAR = 0x01;                                     // Clear counter register.
				app_timer_start(m_freq_meas_timer_id, FREQ_MEAS_TIME , NULL);		    // Timeout 1 sec. Meassure timewindow.
				NRF_TIMER2->TASKS_START = 0x01;                                     // Start counting pulses from 555 timer.               

				advcount = 0;                                                       // Reset counter.
			}						

			
			
			if(advcount==MEASURE_INTERVAL-1)
			{
				nrf_gpio_pin_set(16);                                               // To ensure stable voltage on pin 16, it's set litle earlier.
			}


			
			if(advcount==MEASURE_INTERVAL)
			{
				NRF_GPIOTE->CONFIG[0]   = 0x10f01;                                  //Event on rising edge on pin15.						 
				NRF_TIMER2->TASKS_CLEAR = 0x01; 
				app_timer_start(m_freq_meas_timer_id, FREQ_MEAS_TIME , NULL);		    // Timeout 1 sec.
				NRF_TIMER2->TASKS_START = 0x01;                                     // Start counting.               			                                                          
			}	 
	}
}


	
static void freq_meas_timeout_handler(void * p_context)
{
  	NRF_GPIOTE->CONFIG[0] = 0x0;                                            // Disable gpiote
	  app_timer_stop(m_freq_meas_timer_id);                                   // Stop measuretime timer.
	  nrf_gpio_pin_clear(16);                                                 // Turn off 555 timer.
	  advcount = 0;                                                           // Reset advcounter.
	
	  double temp;
	  
	
    NRF_TIMER2->TASKS_CAPTURE[0] = 1;                                       // Copy freq counter value to cc-register.
	  temp = NRF_TIMER2->CC[0];                   
	  m_service_data_data[0] = (1/temp)*17951-1.994;                          // Normalizing/processing the raw data.                 
	   
	  ble_advdata_set(&m_advdata, NULL);                                      // Update advertising data.
}




void measure_time_cnf()
{
	  app_timer_create(&m_freq_meas_timer_id,APP_TIMER_MODE_REPEATED,freq_meas_timeout_handler);	
}



/**
 * @brief Function for application main entry.
 */
int main(void)
{   
	  
	  
    // Initialize.
	  timer2_cnf();  
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
	  gpio_cnf();
	  GPIOTE_cnf();
	  measure_time_cnf();
    ble_stack_init();
	  ble_radio_notification_init(3, NRF_RADIO_NOTIFICATION_DISTANCE_800US ,radio_notif_handler_evt);
    ppi_cnf(); 
    sd_ble_gap_tx_power_set(TX_POWER);	
	  advertising_init();

    // Start execution.
	  
    advertising_start();
    
    // Enter main loop.
    for (;; )
    {						      
				power_manage();
    }
}
