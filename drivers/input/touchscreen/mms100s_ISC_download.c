//--------------------------------------------------------
//
//
//	Melfas MMS100S V4 Series Download base v1.0 2012.01.18
//

//#include "mms100S_ISC_download.h"
//#include "mms100S_download_porting.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <asm/gpio.h>
#include <asm/io.h>


#include "mcs8000_download.h"

//============================================================
//
//	Include MELFAS Binary code File ( ex> MELFAS_FIRM_bin.c)
//
//	Warning!!!!
//		Please, don't add binary.c file into project
//		Just #include here !!
//
//============================================================

//#include "MELFAS_FIRM_bin.c"
extern void mcsdl_select_binary_data(int hw_ver);
//extern UINT16 binary_nLength; PSJ
UINT16 binary_nLength; //PSJ
// extern UINT8 *binary; //PSJ
UINT8 *binary;

UINT8  ucSlave_addr = ISC_MODE_SLAVE_ADDRESS;

//============================================================
//	I2C Function
//============================================================
extern BOOLEAN mms100s_i2c_write(UINT8 slave_addr, UINT8* buffer, UINT8 packet_len);
extern BOOLEAN mms100s_i2c_read(UINT8 slave_addr, UINT8* buffer, UINT8 packet_len);

static void mms100S_ISC_set_ready(void);
int mms100S_ISC_download_binary_data(void);
static int mms100S_ISC_download(const UINT8 *pBianry, const UINT16 unLength);
static void mms100S_ISC_enter_download_mode(void);
int mms100S_ISC_download(const UINT8 *pBianry, const UINT16 unLength);
static void mms100S_ISC_reboot_mcs(void);
static void mms100S_ISC_Write(UINT8 *_pBinary_reordered, UINT16 _unDownload_size, UINT16 start_addr, UINT16 cLength);
static UINT8 mms100S_ISC_Verify(UINT8 *_pBinary_reordered, UINT16 _unDownload_size, UINT16 start_addr, UINT16 cLength);

extern UINT16 binary_nLength;
extern UINT8 *binary;

//---------------------------------
//	Delay functions
//---------------------------------
//void mcsdl_delay(UINT32 nCount);
extern void mcsdl_delay(UINT32 nCount);


#if MELFAS_ENABLE_DELAY_TEST					// For initial porting test.
void mcsdl_delay_test(INT32 nCount);
#endif


//---------------------------------
//	For debugging display
//---------------------------------
#if MELFAS_ENABLE_DBG_PRINT
static void T_mcsdl_ISC_print_result(int nRet);
#endif


//----------------------------------
// Download enable command
//----------------------------------
#if MELFAS_USE_PROTOCOL_COMMAND_FOR_DOWNLOAD

void melfas_send_download_enable_command(void)
{
	// TO DO : Fill this up

}

#endif

int mms100S_download(void)
{
	int ret = MCSDL_RET_SUCCESS;
	int i=0;

	//PSJ mcsdl_select_binary_data(0x0A);

	mms100S_ISC_set_ready();
        
	for(i=0; i<3 ; i++)
	{
        ret = mms100S_ISC_download_binary_data();    //retry ISC mode download
//          ret = mms100S_ISC_download_binary_file(void)//retry ISC mode download
	if (ret)
        {      
    			printk("<MELFAS> SET Download ISC Fail\n");
        }
		else
			break;
	}
    return ret;    

}


//============================================================
//
//	Main Download furnction
//
//   1. Run mms100S_ISC_download(*pBianry,unLength, nMode)
//       nMode : 0 (Core download)
//       nMode : 1 (Private Custom download)
//       nMode : 2 (Public Custom download)
//
//============================================================

int mms100S_ISC_download_binary_data(void)
{

	int nRet;

#if MELFAS_USE_PROTOCOL_COMMAND_FOR_DOWNLOAD
	melfas_send_download_enable_command();
	mcsdl_delay(MCSDL_DELAY_100US);
#endif

	MELFAS_DISABLE_BASEBAND_ISR();					// Disable Baseband touch interrupt ISR.
	MELFAS_DISABLE_WATCHDOG_TIMER_RESET();			// Disable Baseband watchdog timer

   
	//------------------------
	// Run Download
	//------------------------
	if( binary != NULL && binary_nLength > 0 && binary_nLength < MELFAS_MMS100S_FIRMWARE_MAX_SIZE ){
	}else{
		nRet = T_MCSDL_RET_WRONG_BINARY;
        goto fw_error;
	}
	
    nRet = mms100S_ISC_download( (const UINT8*) binary, (const UINT16)binary_nLength);
    if (nRet)
        goto fw_error;

	MELFAS_ROLLBACK_BASEBAND_ISR(); 				// Roll-back Baseband touch interrupt ISR.
	MELFAS_ROLLBACK_WATCHDOG_TIMER_RESET(); 		// Roll-back Baseband watchdog timer
	return 0;

fw_error:

#if MELFAS_ENABLE_DBG_PRINT
	T_mcsdl_ISC_print_result( nRet );								// Show result
#endif

    
//	mcsdl_erase_flash(0);
//	mcsdl_erase_flash(1);
	return nRet;

}



int mms100S_ISC_download_binary_file(void)
{
	int nRet;
    
	UINT8  *pBinary;
	UINT16 nBinary_length = 0;
	//==================================================
	//
	//	1. Read '.bin file'
	//   2. *pBinary       : Binary data
	//	   nBinary_length : Firmware size
    //   3. Run mms100S_ISC_download(*pBianry,unLength)
	//
	//==================================================

	#if 0

		// TO DO : File Process & Get file Size(== Binary size)
		//			This is just a simple sample

		FILE *fp;
		INT  nRead;

		//------------------------------
		// Open a file
		//------------------------------

		if( fopen( fp, "MELFAS_FIRMWARE.bin", "rb" ) == NULL ){
			return MCSDL_RET_FILE_ACCESS_FAILED;
		}

		//------------------------------
		// Get Binary Size
		//------------------------------

		fseek( fp, 0, SEEK_END );

		nBinary_length = (UINT16)ftell(fp);

		//------------------------------
		// Memory allocation
		//------------------------------

		pBinary = (UINT8*)malloc( (INT)nBinary_length );

		if( pBinary == NULL ){

			return MCSDL_RET_FILE_ACCESS_FAILED;
		}

		//------------------------------
		// Read binary file
		//------------------------------

		fseek( fp, 0, SEEK_SET );

		nRead = fread( pBinary, 1, (INT)nBinary_length, fp );		// Read binary file

		if( nRead != (INT)nBinary_length ){

			fclose(fp);												// Close file

			if( pBinary != NULL )										// free memory alloced.
				free(pBinary);

			return MCSDL_RET_FILE_ACCESS_FAILED;
		}

		//------------------------------
		// Close file
		//------------------------------

		fclose(fp);

	#endif

#if MELFAS_USE_PROTOCOL_COMMAND_FOR_DOWNLOAD
    melfas_send_download_enable_command();
    mcsdl_delay(MCSDL_DELAY_100US);
#endif
    
    MELFAS_DISABLE_BASEBAND_ISR();                  // Disable Baseband touch interrupt ISR.
    MELFAS_DISABLE_WATCHDOG_TIMER_RESET();          // Disable Baseband watchdog timer

	if( pBinary != NULL && nBinary_length > 0 && nBinary_length < MELFAS_MMS100S_FIRMWARE_MAX_SIZE ){
	}else{
		nRet = T_MCSDL_RET_WRONG_BINARY;
        goto fw_error;
	}
    

    //------------------------
    // Run Download
    //------------------------
    
        
    nRet = mms100S_ISC_download( (const UINT8*) pBinary, (const UINT16)nBinary_length);
    if (nRet)
        goto fw_error;
    
    
    MELFAS_ROLLBACK_BASEBAND_ISR();                 // Roll-back Baseband touch interrupt ISR.
    MELFAS_ROLLBACK_WATCHDOG_TIMER_RESET();         // Roll-back Baseband watchdog timer
    return 0;

fw_error:
//  mcsdl_erase_flash(0);
//  mcsdl_erase_flash(1);
    return nRet;

}




//------------------------------------------------------------------
//
//	Sub functions
//
//------------------------------------------------------------------
static void mms100S_ISC_read_data(UINT8 addr, UINT8 *read_buffer, UINT8 read_num)
{
    UINT8  write_buffer;

    write_buffer = addr;
	mms100s_i2c_write(ucSlave_addr, &write_buffer, 1);
    mms100s_i2c_read(ucSlave_addr, read_buffer, read_num);
}

void mms100S_ISC_enter_download_mode(void)
{
    UINT8  write_buffer[4];
    ucSlave_addr = ISC_MODE_SLAVE_ADDRESS;
    write_buffer[0] = ISC_DOWNLOAD_MODE_ENTER; // command
	if(!mms100s_i2c_write(ucSlave_addr,write_buffer, 1))
    {
        printk("<MELFAS> Firmware is not exist.\n");
        mms100S_ISC_reboot_mcs();
    }   
        
}

static UINT8 mms100S_ISC_firmware_erase(void)
{
    UINT8 write_buffer[10];
    UINT8 read_buffer[4] = {0,};
    UINT8 i = 0;
    
    write_buffer[0] = MCSDL_ISC_DEFAULT_ISC_ADDR;
    write_buffer[1] = 0x00;
    write_buffer[2] = 0xC1;
    write_buffer[3] = 0x00;
    write_buffer[4] = 0x00;
	mms100s_i2c_write(ucSlave_addr,write_buffer, 5);


    write_buffer[0] = MCSDL_ISC_DEFAULT_ISC_STATUS_ADDR;
    while (read_buffer[2] != 0x0C)
    {
        mms100s_i2c_write(ucSlave_addr,write_buffer, 1);
        mms100S_ISC_read_data(MCSDL_ISC_DEFAULT_ISC_STATUS_ADDR, read_buffer, 4);

        if (read_buffer[2] == 0x0C)
            printk("<MELFAS> Firmware Mass Erase done.\n");
        else if (read_buffer[2] == 0x08)
            printk("<MELFAS> Firmware Mass Erase enter success!!!\n");

        //PSJ mcsdl_delay(MCSDL_DELAY_1MS);
        if (i > 20) 
                return T_MCSDL_FIRMWARE_MASS_ERASE_FAILED;
        i++;
    }
    return T_MCSDL_RET_SUCCESS;

}

static UINT8 mms100S_ISC_firmware_update(UINT8 *_pBinary_reordered, UINT16 _unDownload_size)
{
    UINT8 bRet = FALSE;

	UINT16 nOffset = 0;
	UINT16 cLength = 4; // word
    UINT16 start_addr = 0;// 0~5119 word
    UINT8 retry_cnt = 0;
    
        
    while( (start_addr * cLength) <= _unDownload_size)
    {

        mms100S_ISC_Write(_pBinary_reordered, _unDownload_size, start_addr, cLength);
		#if 1
        bRet = mms100S_ISC_Verify(_pBinary_reordered, _unDownload_size, start_addr, cLength);
        if(!bRet)
        {
            if(retry_cnt > 5) 
                {
                    printk("<MELFAS> ISC verify failed!!!\n");
                    return T_MCSDL_FIRMWARE_UPDATE_FAILED;
                }
            else
                {
                    printk("<MELFAS> ISC write retry!!!\n");
                    retry_cnt++;
                }
        }
        else
        {
        #endif
            retry_cnt = 0;
            nOffset++;
            start_addr = nOffset * MELFAS_ISC_DATA_SIZE / cLength;
			#if 1
        }
		#endif
    }
    if(bRet)
        printk("<MELFAS> Firmware update success!!!\n");

    return T_MCSDL_RET_SUCCESS;

}
void mms100S_ISC_Write(UINT8 *_pBinary_reordered, UINT16 _unDownload_size, UINT16 start_addr, UINT16 cLength)
{

    int k;    
	UINT8  write_buffer[MELFAS_BUFFER_SIZE]={0xFF,}; //ISC_DATA_SIZE + ISC_DATA_HEADER_SIZE + ISC_DATA_TAIL_SIZE

	printk("%s++++\n",__func__);
	
     write_buffer[0] = MCSDL_ISC_DEFAULT_ISC_ADDR; // ISC Address
     write_buffer[1] = (UINT8) ((start_addr) & 0X00FF); // sub_command
     write_buffer[2] = (UINT8) ((start_addr >> 8) & 0X00FF); // sub_command ;
     write_buffer[3] = 0X00;
     write_buffer[4] = 0X00;

	 printk("<%2x %2x>\n", write_buffer[1], write_buffer[2]);
     
     for (k = 0; k < MELFAS_ISC_DATA_SIZE; k += 4)
     {
         write_buffer[MELFAS_ISC_CMD_HEADER_SIZE + k + 0] = _pBinary_reordered[start_addr * cLength + k + 0];
         write_buffer[MELFAS_ISC_CMD_HEADER_SIZE + k + 1] = _pBinary_reordered[start_addr * cLength + k + 1];
         write_buffer[MELFAS_ISC_CMD_HEADER_SIZE + k + 2] = _pBinary_reordered[start_addr * cLength + k + 2];
         write_buffer[MELFAS_ISC_CMD_HEADER_SIZE + k + 3] = _pBinary_reordered[start_addr * cLength + k + 3];

	 printk("[%d] %02X %02X %02X %02X\n", k,write_buffer[MELFAS_ISC_CMD_HEADER_SIZE + k + 0] , write_buffer[MELFAS_ISC_CMD_HEADER_SIZE + k + 1] , 
	 	write_buffer[MELFAS_ISC_CMD_HEADER_SIZE + k + 2] , write_buffer[MELFAS_ISC_CMD_HEADER_SIZE + k + 3] );

     }
                 
     mms100s_i2c_write( ucSlave_addr, write_buffer,MELFAS_BUFFER_SIZE);

	printk("%s----\n",__func__);	 
    

}

UINT8 mms100S_ISC_Verify(UINT8 *_pBinary_reordered, UINT16 _unDownload_size, UINT16 start_addr, UINT16 cLength)
{

    int k;    
	UINT8  write_buffer[MELFAS_ISC_CMD_HEADER_SIZE]={0xFF,};
	UINT8  read_buffer[MELFAS_ISC_DATA_SIZE]={0xFF,};
	
     write_buffer[0] = MCSDL_ISC_DEFAULT_ISC_ADDR; // ISC Address
     write_buffer[1] = (UINT8) ((start_addr ) & 0X00FF); // sub_command
     write_buffer[2] = 0x40 + (UINT8) ((start_addr>>8) & 0X00FF); // sub_command ;
     write_buffer[3] = 0X00;
     write_buffer[4] = 0X00;
     mms100s_i2c_write( ucSlave_addr, write_buffer,MELFAS_ISC_CMD_HEADER_SIZE);
     mms100s_i2c_read( ucSlave_addr, read_buffer,MELFAS_ISC_DATA_SIZE);

     
     for (k = 0; k < MELFAS_ISC_DATA_SIZE; k += 4)
     {
        if(read_buffer[k + 0] != _pBinary_reordered[start_addr * cLength + k + 0]
            || read_buffer[k + 1] != _pBinary_reordered[start_addr * cLength + k + 1]
            || read_buffer[k + 2] != _pBinary_reordered[start_addr * cLength + k + 2]
            || read_buffer[k + 3] != _pBinary_reordered[start_addr * cLength + k + 3])
            {
            printk("<MELFAS> Verify failed. 0X%0004X\n", (UINT16)(start_addr * cLength + k));
            printk("<MELFAS> orig data 0X%02X %02X %02X %02X\n", 
                _pBinary_reordered[start_addr * cLength + k + 0], _pBinary_reordered[start_addr * cLength + k + 1],
                _pBinary_reordered[start_addr * cLength + k + 2], _pBinary_reordered[start_addr * cLength + k + 3]);
            printk("<MELFAS> read data 0X%02X %02X %02X %02X\n", read_buffer[k + 0], read_buffer[k + 1], read_buffer[k + 2], read_buffer[k + 3]);
            return FALSE;
            }      
		            //printk("<MELFAS> Verify failed. 0X%0004X", (UINT16)(start_addr * cLength + k));
	      printk("<<<<<<<<\n");
            printk("<MELFAS> orig data 0X%02X %02X %02X %02X\n", 
                _pBinary_reordered[start_addr * cLength + k + 0], _pBinary_reordered[start_addr * cLength + k + 1],
                _pBinary_reordered[start_addr * cLength + k + 2], _pBinary_reordered[start_addr * cLength + k + 3]);
            printk("<MELFAS> read data 0X%02X %02X %02X %02X\n", read_buffer[k + 0], read_buffer[k + 1], read_buffer[k + 2], read_buffer[k + 3]);
	      printk(">>>>>>>>\n");

     }    
     return TRUE;

}


/*
void mms100S_ISC_set_ready(void)
{
	//--------------------------------------------
	// Tkey module reset
	//--------------------------------------------

	MCSDL_VDD_SET_LOW(); // power 

	MCSDL_RESETB_SET_INPUT();

	MCSDL_CE_SET_HIGH;
	MCSDL_CE_SET_OUTPUT();
    
	mcsdl_delay(MCSDL_DELAY_60MS);						// Delay for Stable VDD

	MCSDL_VDD_SET_HIGH();

	mcsdl_delay(MCSDL_DELAY_500MS);						// Delay for Stable VDD
}
*/

void mms100S_ISC_set_ready(void)
{
	//--------------------------------------------
	// Tkey module reset
	//--------------------------------------------

	MCSDL_VDD_SET_LOW(); // power 

	//MCSDL_SET_GPIO_I2C();

	MCSDL_GPIO_SDA_SET_OUTPUT(1);
	MCSDL_GPIO_SDA_SET_HIGH();

	MCSDL_GPIO_SCL_SET_OUTPUT(1);
	MCSDL_GPIO_SCL_SET_HIGH();

	MCSDL_RESETB_SET_INPUT();

	//MCSDL_CE_SET_HIGH;
	//MCSDL_CE_SET_OUTPUT();
	//PSJ mcsdl_delay(MCSDL_DELAY_60MS);						// Delay for Stable VDD

	MCSDL_VDD_SET_HIGH();

	//PSJ mcsdl_delay(MCSDL_DELAY_60MS);						// Delay for Stable VDD

}

void mms100S_ISC_reboot_mcs(void)
{
	//--------------------------------------------
	// Tkey module reset
	//--------------------------------------------
    mms100S_ISC_set_ready();
}


//============================================================
//
//	Debugging print functions.
//
//============================================================

#ifdef MELFAS_ENABLE_DBG_PRINT

static void T_mcsdl_ISC_print_result(int nRet)
{
	if( nRet == MCSDL_RET_SUCCESS ){

		printk("<MELFAS> Firmware downloading SUCCESS.\n");

	}else{

		printk("<MELFAS> Firmware downloading FAILED  :  ");

		switch( nRet ){

			case T_MCSDL_RET_SUCCESS                  		:   printk("<MELFAS> MCSDL_RET_SUCCESS\n" );                 		break;
			case T_MCSDL_FIRMWARE_MASS_ERASE_FAILED           :   printk("<MELFAS> MCSDL_FIRMWARE_MASS_ERASE_FAILED\n" );         break;            
			case T_MCSDL_FIRMWARE_UPDATE_MODE_ENTER_FAILED    :   printk("<MELFAS> MCSDL_FIRMWARE_UPDATE_MODE_ENTER_FAILED\n" );	break;
            case T_MCSDL_FIRMWARE_UPDATE_FAILED               :   printk("<MELFAS> MCSDL_FIRMWARE_UPDATE_FAILED\n" );         	break;
    
			case T_MCSDL_RET_PROGRAM_SIZE_IS_WRONG			:   printk("<MELFAS> MCSDL_RET_PROGRAM_SIZE_IS_WRONG\n" );    		break;
			case T_MCSDL_RET_VERIFY_SIZE_IS_WRONG				:   printk("<MELFAS> MCSDL_RET_VERIFY_SIZE_IS_WRONG\n" );      		break;
			case T_MCSDL_RET_WRONG_BINARY						:   printk("<MELFAS> MCSDL_RET_WRONG_BINARY\n" );      				break;
            case T_MCSDL_FIRMWARE_VERSION_INFO_IS_WRONG	    :   printk("<MELFAS> MCSDL_FIRMWARE_VERSION_INFO_IS_WRONG\n" );     break;

			case T_MCSDL_RET_READING_HEXFILE_FAILED       	:   printk("<MELFAS> MCSDL_RET_READING_HEXFILE_FAILED\n" );			break;
			case T_MCSDL_RET_FILE_ACCESS_FAILED       		:   printk("<MELFAS> MCSDL_RET_FILE_ACCESS_FAILED\n" );				break;
			case T_MCSDL_RET_MELLOC_FAILED     		  		:   printk("<MELFAS> MCSDL_RET_MELLOC_FAILED\n" );      			break;
                            
			case T_MCSDL_RET_WRONG_MODULE_REVISION     		:   printk("<MELFAS> MCSDL_RET_WRONG_MODULE_REVISION\n" );      	break;

			default                             			:	printk("<MELFAS> UNKNOWN ERROR. [0x%02X].\n", nRet );      		break;
		}

		printk("\n");
	}

}

#endif

#if 0

//============================================================
//
//	I2C Function
//
//============================================================
BOOLEAN mms100s_i2c_write(UINT8 slave_addr, UINT8* buffer, UINT8 packet_len)
{
    // Please, Use your i2c write function
    //i2c_master_send(client_tmp, (const char*)buffer, (int)packet_len);
    	int ret;
	struct i2c_adapter *adap = client_tmp->adapter;
	struct i2c_msg msg;

	msg.addr = client_tmp->addr;
	msg.flags = client_tmp->flags & I2C_M_TEN;
	msg.len = (int)packet_len;
	msg.buf = (char *)buffer;

	ret = i2c_transfer(adap, &msg, 1);

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	   transmitted, else error code. */
	return (ret == 1) ? count : ret;

}

BOOLEAN mms100s_i2c_read(UINT8 slave_addr, UINT8* buffer, UINT8 packet_len)
{
    // Please, Use your i2c read function
    i2c_master_recv(client_tmp, (const char*)buffer, (int)packet_len);
}

#endif

/*
//============================================================
//
//	Delay Function
//
//============================================================
void mcsdl_delay(UINT32 nCount)
{
    delay_usec(nCount);           // Please, Use your delay function
}
*/

#if MELFAS_ENABLE_DELAY_TEST

//============================================================
//
//	For initial testing of delay and gpio control
//
//	You can confirm GPIO control and delay time by calling this function.
//
//============================================================

void mcsdl_delay_test(INT32 nCount)
{
	INT16 i;

	MELFAS_DISABLE_BASEBAND_ISR();					// Disable Baseband touch interrupt ISR.
	MELFAS_DISABLE_WATCHDOG_TIMER_RESET();			// Disable Baseband watchdog timer

	//--------------------------------
	//	Repeating 'nCount' times
	//--------------------------------
    MCSDL_VDD_SET_LOW()

	MCSDL_RESETB_SET_OUTPUT();

	MCSDL_RESETB_SET_HIGH();

    for (i = 0; i < nCount; i++)
    {
		MCSDL_RESETB_SET_LOW();
		mcsdl_delay(MCSDL_DELAY_100US);
		MCSDL_RESETB_SET_HIGH();
		mcsdl_delay(MCSDL_DELAY_100US);
	}
	MELFAS_ROLLBACK_BASEBAND_ISR();					// Roll-back Baseband touch interrupt ISR.
	MELFAS_ROLLBACK_WATCHDOG_TIMER_RESET();			// Roll-back Baseband watchdog timer
}


#endif


//------------------------------------------------------------------
//
//	Download function
//
//------------------------------------------------------------------

int mms100S_ISC_download(const UINT8 *pBianry, const UINT16 unLength)
{
	int nRet;
    int i=0;
        

	//---------------------------------
	// Make it ready
	//---------------------------------
#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
	printk("<MELFAS> Ready\n");
#endif

	#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
    printk("<MELFAS> Firmware_download_via_ISC start!!!\n");
	#endif

    //--------------------------------------------------------------
    // ISC DOWNLOAD MODE ENTER
    //--------------------------------------------------------------
    printk("<MELFAS> ISC_DOWNLOAD_MODE_ENTER\n\n");            
    mms100S_ISC_enter_download_mode();
//	mcsdl_delay(MCSDL_DELAY_100MS);
    ucSlave_addr = ISC_DEFAULT_SLAVE_ADDR;

    //--------------------------------------------------------------
    // FIRMWARE MASS ERASE
    //--------------------------------------------------------------
    printk("<MELFAS> FIRMWARE_MASS_ERASE\n\n");            
    nRet = mms100S_ISC_firmware_erase();
    if(nRet != T_MCSDL_RET_SUCCESS) goto MCSDL_DOWNLOAD_FINISH;


     //--------------------------------------------------------------
    // FIRMWARE UPDATE 
    //--------------------------------------------------------------
	printk("<MELFAS> FIRMWARE UPDATE\n\n");            
    nRet = mms100S_ISC_firmware_update((UINT8 *)pBianry, (UINT16)unLength);
    if(nRet != T_MCSDL_RET_SUCCESS) goto MCSDL_DOWNLOAD_FINISH;

	nRet = T_MCSDL_RET_SUCCESS;


MCSDL_DOWNLOAD_FINISH :

	#if MELFAS_ENABLE_DBG_PRINT
	T_mcsdl_ISC_print_result( nRet );								// Show result
	#endif


	#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
	printk("<MELMAS> Rebooting\n");
	printk("<MELMAS>  - Fin.\n\n");
	#endif

	mms100S_ISC_reboot_mcs();


	return nRet;
}



