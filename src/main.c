

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"


#include "iot_network_manager_private.h"
#include "iot_wifi.h"
#include "iot_init.h"
#include "iot_config.h"
#include "iot_demo_logging.h"

#include "platform/iot_threads.h"

#include "driver/uart.h"
#include "iot_uart.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "aws_clientcredential.h"
#include "aws_clientcredential_keys.h"

#include "aws_demo_config.h"
//#include "aws_greengrass_discovery.h"
//#include "types/iot_network_types.h"
//#include "platform/iot_threads.h"
static uint32_t demoConnectedNetwork = AWSIOT_NETWORK_TYPE_NONE;

#define democonfigMQTT_ECHO_TASK_STACK_SIZE                          ( configMINIMAL_STACK_SIZE * 2 )


/* Logging Task Defines. */
#define mainLOGGING_MESSAGE_QUEUE_LENGTH    ( 32 )
#define mainLOGGING_TASK_STACK_SIZE         ( configMINIMAL_STACK_SIZE * 4 )
extern void vApplicationMallocFailedHook( void );
extern void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                        char * pcTaskName );
/**
 * @brief Initializes the board.
 */
static void prvMiscInitialization( void );


/* Semaphore used to wait for a network to be available. */
static IotSemaphore_t demoNetworkSemaphore;
//static IotNetworkManagerSubscription_t subscription = IOT_NETWORK_MANAGER_SUBSCRIPTION_INITIALIZER;



IotUARTHandle_t xConsoleUart;


static void iot_uart_init( void )
{
    IotUARTConfig_t xUartConfig;
    int32_t status = IOT_UART_SUCCESS;
    
    xConsoleUart = iot_uart_open( UART_NUM_0 );
    configASSERT( xConsoleUart );
    
    status = iot_uart_ioctl( xConsoleUart, eUartGetConfig, &xUartConfig );
    configASSERT( status == IOT_UART_SUCCESS );
    
    xUartConfig.ulBaudrate = 115200;
    xUartConfig.xParity = eUartParityNone;
    xUartConfig.xStopbits = eUartStopBitsOne;
    xUartConfig.ucFlowControl = true;

    status = iot_uart_ioctl( xConsoleUart, eUartSetConfig, &xUartConfig );
    configASSERT( status == IOT_UART_SUCCESS );
}
/*-----------------------------------------------------------*/


/**
 * @brief Initialize the common libraries, Mqtt library and network manager.
 *
 * @return `EXIT_SUCCESS` if all libraries were successfully initialized;
 * `EXIT_FAILURE` otherwise.
 */
static int _initialize(void)
{
    int status = EXIT_SUCCESS;
    bool commonLibrariesInitialized = false;
    bool semaphoreCreated = false;

    /* Initialize the C-SDK common libraries. This function must be called
     * once (and only once) before calling any other C-SDK function. */
    if( IotSdk_Init() == true )
    {
        commonLibrariesInitialized = true;
    }
    else
    {
        IotLogInfo( "Failed to initialize the common library." );
        status = EXIT_FAILURE;
    }

    if( status == EXIT_SUCCESS )
    {
        if( AwsIotNetworkManager_Init() != pdTRUE )
        {
            IotLogError( "Failed to initialize network manager library." );
            status = EXIT_FAILURE;
        }
    }

    if( status == EXIT_SUCCESS )
    {
        /* Create semaphore to signal that a network is available for the demo. */
        if( IotSemaphore_Create( &demoNetworkSemaphore, 0, 1 ) != true )
        {
            IotLogError( "Failed to create semaphore to wait for a network connection." );
            status = EXIT_FAILURE;
        }
        else
        {
            semaphoreCreated = true;
        }
    }

    /* Initialize all the  networks configured for the device. */
    if( status == EXIT_SUCCESS )
    {
        if( AwsIotNetworkManager_EnableNetwork( configENABLED_NETWORKS ) != configENABLED_NETWORKS )
        {
            IotLogError( "Failed to initialize all the networks configured for the device." );
            status = EXIT_FAILURE;
        }
    }

    if( status == EXIT_FAILURE )
    {
        if( semaphoreCreated == true )
        {
            IotSemaphore_Destroy( &demoNetworkSemaphore );
        }

        if( commonLibrariesInitialized == true )
        {
            IotSdk_Cleanup();
        }
    }

    return status;

}

/*-----------------------------------------------------------*/

int discoverGreengrassCore();



void cal(void)
{
        const IotNetworkInterface_t * pNetworkInterface = NULL;
        void * pConnectionParams = NULL, * pCredentials = NULL;
        pNetworkInterface = AwsIotNetworkManager_GetNetworkInterface( demoConnectedNetwork );
        pConnectionParams = AwsIotNetworkManager_GetConnectionParams( demoConnectedNetwork );
        pCredentials = AwsIotNetworkManager_GetCredentials( demoConnectedNetwork );     
while (1) {
/*    RunCoreMqttMutualAuthDemo( true,
                           clientcredentialIOT_THING_NAME,
                           pConnectionParams,
                           pCredentials,
                           pNetworkInterface );*/
           vStartGreenGrassDiscoveryTask( true,
                           clientcredentialIOT_THING_NAME,
                           pConnectionParams,
                           pCredentials,
                           pNetworkInterface );
			
     //       vTaskDelay( pdMS_TO_TICKS( 1000 ) );    

//            discoverGreengrassCore();
//            printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
            vTaskDelay( pdMS_TO_TICKS( 1000 ) );
}
		
}



/**
 * @brief Application runtime entry point.
 */
void app_main(void)
{
    /* Perform any hardware initialization that does not require the RTOS to be
     * running.  */

    prvMiscInitialization();


    if( SYSTEM_Init() == pdPASS )
    {
        /* A simple example to demonstrate key and certificate provisioning in
         * microcontroller flash using PKCS#11 interface. This should be replaced
         * by production ready key provisioning mechanism. */
        vDevModeKeyProvisioning();


	    ESP_ERROR_CHECK( esp_bt_controller_mem_release( ESP_BT_MODE_CLASSIC_BT ) );
	    ESP_ERROR_CHECK( esp_bt_controller_mem_release( ESP_BT_MODE_BLE ) );


		_initialize();

        printf("Successfully_Successfully_Successfully_Successfully_Successfully_Successfully_Successfully\n");

		//vTaskDelay(5000 / portTICK_PERIOD_MS);

        int status2 = EXIT_SUCCESS;
      /*  const IotNetworkInterface_t * pNetworkInterface = NULL;
        void * pConnectionParams = NULL, * pCredentials = NULL;
        pNetworkInterface = AwsIotNetworkManager_GetNetworkInterface( demoConnectedNetwork );
        pConnectionParams = AwsIotNetworkManager_GetConnectionParams( demoConnectedNetwork );
        pCredentials = AwsIotNetworkManager_GetCredentials( demoConnectedNetwork );
*/
            
        if( xTaskCreate( cal,
                         "Greengrass",
                         democonfigGREENGRASS_DISCOVERY_TASK_STACK_SIZE,
                         NULL,
                         democonfigDEMO_PRIORITY,
                         NULL ) != pdPASS )
        {
            IotLogWarn( "Failed to create thread." );

        }
                

//            discoverGreengrassCore();
     /*   if( xTaskCreate( cal,
                         "mqtt",
                         democonfigMQTT_ECHO_TASK_STACK_SIZE,
                         NULL,
                         democonfigDEMO_PRIORITY,
                         NULL ) != pdPASS )
        {
            // Task creation failed. 
            IotLogWarn( "Failed to create thread." );

        }else{
            IotLogInfo("entered the create class and should created");
        }*/
        /*

        RunCoreMqttMutualAuthDemo( true,
            clientcredentialIOT_THING_NAME,
            pConnectionParams,
            pCredentials,
            pNetworkInterface );
        */


     //   vTaskDelay( pdMS_TO_TICKS( 1000 ) );
         /* Log the demo status. */
        if( status2 == EXIT_SUCCESS )
        {
            /* DO NOT EDIT - This message is used in the test framework to
             * determine whether or not the demo was successful. */
            IotLogInfo( "Demo completed successfully." );
        }
        else
        {
            printf("from here");
            IotLogError( "Error running demo." );
        }

	}


}

/*-----------------------------------------------------------*/


/**
 * @brief Initializes the board.
 */
static void prvMiscInitialization( void )
{
    int32_t uartRet;
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();

    if( ( ret == ESP_ERR_NVS_NO_FREE_PAGES ) || ( ret == ESP_ERR_NVS_NEW_VERSION_FOUND ) )
    {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK( ret );

    iot_uart_init();


    /* Create tasks that are not dependent on the WiFi being initialized. */
    xLoggingTaskInitialize( mainLOGGING_TASK_STACK_SIZE,
                            tskIDLE_PRIORITY + 5,
                            mainLOGGING_MESSAGE_QUEUE_LENGTH );


    configPRINTF( ("Initializing lwIP TCP stack\r\n") );
    esp_netif_init();

}

/*-----------------------------------------------------------*/






/*
 * Hook Functions:
 * https://www.freertos.org/a00016.html
 */


extern void esp_vApplicationTickHook();
void IRAM_ATTR vApplicationTickHook()
{
    esp_vApplicationTickHook();
}

/*-----------------------------------------------------------*/


extern void esp_vApplicationIdleHook();
void vApplicationIdleHook()
{
    esp_vApplicationIdleHook();
}

/*-----------------------------------------------------------*/


void vApplicationDaemonTaskStartupHook( void )
{
}

/*-----------------------------------------------------------*/




//void vApplicationMallocFailedHook( void );
/*
void vApplicationMallocFailedHook( void )
{
    printf( "vApplicationMallocFailedHook() calledn" );
}

*/