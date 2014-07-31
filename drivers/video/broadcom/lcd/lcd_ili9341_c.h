

DISPCTRL_REC_T ili9341_panel01_Init[] = {
     /* + Sleep out */
     {DISPCTRL_WR_CMND         , 0x11                           , 0       },   
     {DISPCTRL_SLEEP_MS        , 0                              , 120      },
     /* - Sleep out */


     /* + Initializing Sequnece */
    {DISPCTRL_WR_CMND         , 0xCF                            , 0        },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0xF9    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x30    },    

    {DISPCTRL_WR_CMND         , 0xED                            , 0        },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x64    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x03    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x12    },  
    {DISPCTRL_WR_DATA         , 0x00                            , 0x81    },  

    {DISPCTRL_WR_CMND         , 0xE8                            , 0        },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x85    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x78    },  

    {DISPCTRL_WR_CMND         , 0xCB                            , 0        },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x39    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x2D    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },  
    {DISPCTRL_WR_DATA         , 0x00                            , 0x34    }, 
    {DISPCTRL_WR_DATA         , 0x00                            , 0x02    },  

    {DISPCTRL_WR_CMND_DATA    , 0xF7                        , 0x20    },
			
    {DISPCTRL_WR_CMND         , 0xEA                           , 0         },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },      

    {DISPCTRL_WR_CMND         , 0xB6                           , 0x0       },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0A    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0xC2    },  /* A2 --> C2 flip the display*/
    {DISPCTRL_WR_DATA         , 0x00                            , 0x27    },  
	
    {DISPCTRL_WR_CMND_DATA    , 0x35                        , 0x00    },
    {DISPCTRL_WR_CMND_DATA    , 0x36                        , 0x08    },
    {DISPCTRL_WR_CMND_DATA    , 0x3A                        , 0x66    }, /* RGB565 : 0x06 RGB666:0x06 */

    {DISPCTRL_WR_CMND         , 0xB1                            , 0x0     },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x16    }, 

    {DISPCTRL_WR_CMND         , 0xB4                            , 0x0     },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },            

    {DISPCTRL_WR_CMND_DATA    , 0xF2                        ,  0x02   },

    {DISPCTRL_WR_CMND_DATA    , 0x26                        , 0x01    },

    {DISPCTRL_WR_CMND         , 0xF6                            , 0x0     },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x01    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x30    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },

    {DISPCTRL_WR_CMND         , 0xB0                            , 0x0     },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0xE0    },     
    /* - Initializing Sequnece */


    /* + Power Setting Sequence */
    {DISPCTRL_WR_CMND_DATA    , 0xC0                        , 0x16    },
    {DISPCTRL_WR_CMND_DATA    , 0xC1                        , 0x10    },

    {DISPCTRL_WR_CMND         , 0xC5                            , 0        },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x30    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x3D    },  
	     	
    {DISPCTRL_WR_CMND         , 0x2A                            , 0x0     },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0xEF    },      


    {DISPCTRL_WR_CMND         , 0x2B                            , 0x0     },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x01    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x3F    },      
    /* - Power Setting Sequence */
		
    /* + Gama setting */
    {DISPCTRL_WR_CMND         , 0xE0                            , 0x0     },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0F    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x1F    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x1D    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0A    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0F    },

    {DISPCTRL_WR_DATA         , 0x00                            , 0x0C    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x50    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x72    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x3E    },	
    {DISPCTRL_WR_DATA         , 0x00                            , 0x08    },
            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x14    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x08    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x12    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0C    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },
	
    {DISPCTRL_WR_CMND         , 0xE1                            , 0x0     },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0C    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0F    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x04    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x10    },

    {DISPCTRL_WR_DATA         , 0x00                            , 0x06    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x27    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x21    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x37    },	
    {DISPCTRL_WR_DATA         , 0x00                            , 0x02    }, 
         
    {DISPCTRL_WR_DATA         , 0x00                            , 0x09    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x08    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x24    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x2A    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x03    },
    /* - Gama setting */

	
    /* + Display On */
    {DISPCTRL_WR_CMND         , 0x29                           , 0          },   
    /* - Display On */

    /* + RAM Access setting */
    {DISPCTRL_WR_CMND         , 0x2C                           , 0          }, 
    /* - RAM Access setting */


    /* --- END OF COMMAND LIST ---*/
    {DISPCTRL_LIST_END       , 0                                 , 0            }
};

DISPCTRL_REC_T ili9341_panel01_Sleep[] = {
    {DISPCTRL_WR_CMND         , 0x10                           , 0           },   
    {DISPCTRL_LIST_END       , 0                                   , 0          }
};



/* Panel 2 */
DISPCTRL_REC_T ili9341_panel02_Init[] = {

    {DISPCTRL_WR_CMND         , 0xCF                            , 0        },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0xE3    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x30    },     
	
    {DISPCTRL_WR_CMND         , 0xED                            , 0        },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x64    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x03    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x12    },  
    {DISPCTRL_WR_DATA         , 0x00                            , 0x81    },  

    {DISPCTRL_WR_CMND         , 0xCB                            , 0        },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x39    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x2C    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },  
    {DISPCTRL_WR_DATA         , 0x00                            , 0x34    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x02    },  
	
    {DISPCTRL_WR_CMND_DATA    , 0xF7                        , 0x20    },
		
    {DISPCTRL_WR_CMND         , 0xEA                            , 0        },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },            

    {DISPCTRL_WR_CMND_DATA    , 0xC0                        , 0x1B    },
    {DISPCTRL_WR_CMND_DATA    , 0xC1                        , 0x11    },

    {DISPCTRL_WR_CMND         , 0xC5                            , 0        },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x22    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x3D    },  

    {DISPCTRL_WR_CMND_DATA    , 0xC7                        , 0x00    },   

    {DISPCTRL_WR_CMND         , 0xE8                            , 0        },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x85    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x78    }, 

	
     /* Initializing sequence */
    {DISPCTRL_WR_CMND_DATA    , 0x35                        , 0x00     },
    {DISPCTRL_WR_CMND_DATA    , 0x36                        , 0x08     },
    {DISPCTRL_WR_CMND_DATA    , 0x3A                        , 0x06     }, //RGB565, RGB666:0x06h

    {DISPCTRL_WR_CMND         , 0xB1                            , 0x0      },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x17     },  /* Frame Rtate 0x17 == 83Hz */
	
    {DISPCTRL_WR_CMND         , 0xB5                           , 0x0       },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x02     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x02     },  
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0A     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x14     },  
   
    {DISPCTRL_WR_CMND         , 0xB6                           , 0x0       },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0A     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0xD2     },  

    {DISPCTRL_WR_CMND         , 0xF6                            , 0x0      },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x01     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x30     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00     },


     /*Gama setting sequence */
    {DISPCTRL_WR_CMND_DATA    , 0xF2                         , 0x02    },

    {DISPCTRL_WR_CMND_DATA    , 0x26                         , 0x01    },

    {DISPCTRL_WR_CMND         , 0xE0                            , 0x0      },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0F     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x25     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x20     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0C     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0E     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x07     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x52     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0xA4     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x47     },	
    {DISPCTRL_WR_DATA         , 0x00                            , 0x09     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x16     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x07     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x1F     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0F     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00     },
	
    {DISPCTRL_WR_CMND         , 0xE1                            , 0x0      },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x1A     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x1E     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x03     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0D     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x05     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x30     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x62     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x41     },	
    {DISPCTRL_WR_DATA         , 0x00                            , 0x05     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0E     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x08     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x20     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x30     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0F     },

     /* Sleep out sequence */
    {DISPCTRL_WR_CMND         , 0x11                           , 0          },   
    {DISPCTRL_SLEEP_MS         , 0                                , 120      },
	
    {DISPCTRL_WR_CMND         , 0x29                           , 0          },   

    /* ------------ END OF COMMAND LIST -------------- */
    {DISPCTRL_LIST_END         , 0                                 , 0          }
};

DISPCTRL_REC_T ili9341_panel02_Sleep[] = {
    {DISPCTRL_SLEEP_MS        , 0                               , 10          },
    {DISPCTRL_LIST_END       , 0                                 , 0           }
};

DISPCTRL_REC_T ili9341_panel03_Init[] = {
	{DISPCTRL_WR_CMND         , 0xCF                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0xF9    },	  
	{DISPCTRL_WR_DATA         , 0                               , 0x30    },
	
	{DISPCTRL_WR_CMND         , 0xED                            , 0       },
	{DISPCTRL_WR_DATA         , 0                               , 0x64    },
	{DISPCTRL_WR_DATA         , 0                               , 0x03    },	  
	{DISPCTRL_WR_DATA         , 0                               , 0x12    },
	{DISPCTRL_WR_DATA         , 0                               , 0x81    },
	
	{DISPCTRL_WR_CMND         , 0xCB                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x39    },
	{DISPCTRL_WR_DATA         , 0                               , 0x2C    },	  
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x34    },
	{DISPCTRL_WR_DATA         , 0                               , 0x02    },
	
	{DISPCTRL_WR_CMND         , 0xF7                            , 0       },
	{DISPCTRL_WR_DATA         , 0                               , 0x20    },
	
	{DISPCTRL_WR_CMND         , 0xEA                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	
	{DISPCTRL_WR_CMND         , 0xC0                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x1D    },
	
	{DISPCTRL_WR_CMND         , 0xC1                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x10    },
	
	{DISPCTRL_WR_CMND         , 0xC5                            , 0       },
	{DISPCTRL_WR_DATA         , 0                               , 0x3C    },
	{DISPCTRL_WR_DATA         , 0                               , 0x2A    },
	
	{DISPCTRL_WR_CMND         , 0xC7                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	
	{DISPCTRL_WR_CMND         , 0xE8                            , 0},
	{DISPCTRL_WR_DATA         , 0                               , 0x85    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },	  
	{DISPCTRL_WR_DATA         , 0                               , 0x78    },
	
	//Initializing sequence
	{DISPCTRL_WR_CMND         , 0x35                            , 0       },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	
	{DISPCTRL_WR_CMND         , 0x36                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0xD8    },
	
	{DISPCTRL_WR_CMND         , 0x3A                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x06    },
	
	{DISPCTRL_WR_CMND         , 0xB1                            , 0       },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x16    },
	  
	{DISPCTRL_WR_CMND         , 0xB5                            , 0       },
	{DISPCTRL_WR_DATA         , 0                               , 0x04    },
	{DISPCTRL_WR_DATA         , 0                               , 0x04    },
	{DISPCTRL_WR_DATA         , 0                               , 0x0A    },
	{DISPCTRL_WR_DATA         , 0                               , 0x14    },
	
	{DISPCTRL_WR_CMND         , 0xB6                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x0A    },
	{DISPCTRL_WR_DATA         , 0                               , 0xC2    },
	
	{DISPCTRL_WR_CMND         , 0xF6                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x01    },
	{DISPCTRL_WR_DATA         , 0                               , 0x30    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	
	//Gama setting
	{DISPCTRL_WR_CMND         , 0xF2                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x03    },
	  
	{DISPCTRL_WR_CMND         , 0xE0                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x0F    },
	{DISPCTRL_WR_DATA         , 0                               , 0x24    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x20    },
	{DISPCTRL_WR_DATA         , 0                               , 0x0F    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x0F    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x0B    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x50    },
	{DISPCTRL_WR_DATA         , 0                               , 0xA4    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x3E    },
	{DISPCTRL_WR_DATA         , 0                               , 0x06    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x10    },
	{DISPCTRL_WR_DATA         , 0                               , 0x01    },
	{DISPCTRL_WR_DATA         , 0                               , 0x1A    },
	{DISPCTRL_WR_DATA         , 0                               , 0x19    },
	{DISPCTRL_WR_DATA         , 0                               , 0x0F    },
	
	{DISPCTRL_WR_CMND         , 0xE1                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x18    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x19    },
	{DISPCTRL_WR_DATA         , 0                               , 0x03    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x0F    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x04    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x31    },
	{DISPCTRL_WR_DATA         , 0                               , 0x53    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x46    },
	{DISPCTRL_WR_DATA         , 0                               , 0x09    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x12    },
	{DISPCTRL_WR_DATA         , 0                               , 0x0F    },
	{DISPCTRL_WR_DATA         , 0                               , 0x26    },
	{DISPCTRL_WR_DATA         , 0                               , 0x2A    },
	{DISPCTRL_WR_DATA         , 0                               , 0x03    },
	
	{DISPCTRL_WR_CMND         , 0xE2                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x08    },
	{DISPCTRL_WR_DATA         , 0                               , 0x09    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x09    },
	{DISPCTRL_WR_DATA         , 0                               , 0x09    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x09    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x09    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x09    },
	{DISPCTRL_WR_DATA         , 0                               , 0x09    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x09    },
	{DISPCTRL_WR_DATA         , 0                               , 0x09    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x09    },
	{DISPCTRL_WR_DATA         , 0                               , 0x09    },
	{DISPCTRL_WR_DATA         , 0                               , 0x09    },
	{DISPCTRL_WR_DATA         , 0                               , 0x09    },
	{DISPCTRL_WR_DATA         , 0                               , 0x88    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x88    },
	
	{DISPCTRL_WR_CMND         , 0xE3                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x06    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x06    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x07    },
	{DISPCTRL_WR_DATA         , 0                               , 0x03    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x05    },
	{DISPCTRL_WR_DATA         , 0                               , 0x04    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x04    },
	{DISPCTRL_WR_DATA         , 0                               , 0x04    },
	{DISPCTRL_WR_DATA         , 0                               , 0x04    },
	{DISPCTRL_WR_DATA         , 0                               , 0x04    },
	{DISPCTRL_WR_DATA         , 0                               , 0x02    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x02    },
	{DISPCTRL_WR_DATA         , 0                               , 0x01    },
	{DISPCTRL_WR_DATA         , 0                               , 0x01    },
	{DISPCTRL_WR_DATA         , 0                               , 0x01    },
	{DISPCTRL_WR_DATA         , 0                               , 0x01    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },		
	
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x80    },
	{DISPCTRL_WR_DATA         , 0                               , 0x80    },
	{DISPCTRL_WR_DATA         , 0                               , 0x80    },
	{DISPCTRL_WR_DATA         , 0                               , 0x80    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x82    },
	{DISPCTRL_WR_DATA         , 0                               , 0x82    },
	{DISPCTRL_WR_DATA         , 0                               , 0x82    },
	{DISPCTRL_WR_DATA         , 0                               , 0x82    },

	//Sleep out
	{DISPCTRL_WR_CMND         , 0x11                            , 0       },
	
	{DISPCTRL_SLEEP_MS        , 0                               , 130     },
	
	{DISPCTRL_WR_CMND         , 0x29                            , 0       },

	{DISPCTRL_SLEEP_MS        , 0                               , 40      },
	  
	//--- END OF COMMAND LIST -----------------------
	{DISPCTRL_LIST_END        , 0                               , 0       }
};


DISPCTRL_REC_T ili9341_panel03_Sleep[] = {
#if 0 //Cori Model
	{DISPCTRL_WR_CMND         , 0x10                            , 0       },
	{DISPCTRL_SLEEP_MS        , 0                               , 120     },
	{DISPCTRL_LIST_END        , 0                               , 0       }
#endif

    {DISPCTRL_WR_CMND         , 0x10                            , 0       },
    {DISPCTRL_LIST_END        , 0                               , 0       }
};


DISPCTRL_REC_T ili9341_panel04_Init[] = {
	{DISPCTRL_WR_CMND         , 0xCF                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0xFA    },	  
	{DISPCTRL_WR_DATA         , 0                               , 0xF0    },
	
	{DISPCTRL_WR_CMND         , 0xED                            , 0       },
	{DISPCTRL_WR_DATA         , 0                               , 0x64    },
	{DISPCTRL_WR_DATA         , 0                               , 0x03    },	  
	{DISPCTRL_WR_DATA         , 0                               , 0x12    },
	{DISPCTRL_WR_DATA         , 0                               , 0x81    },
	
	{DISPCTRL_WR_CMND         , 0xCB                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x39    },
	{DISPCTRL_WR_DATA         , 0                               , 0x2C    },	  
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x33    },
	{DISPCTRL_WR_DATA         , 0                               , 0x06    },
	
	{DISPCTRL_WR_CMND         , 0xF7                            , 0       },
	{DISPCTRL_WR_DATA         , 0                               , 0x20    },
	
	{DISPCTRL_WR_CMND         , 0xEA                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	
	{DISPCTRL_WR_CMND         , 0xC0                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x1F    },
	
	{DISPCTRL_WR_CMND         , 0xC1                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x12    },
	
	{DISPCTRL_WR_CMND         , 0xC5                            , 0       },
	{DISPCTRL_WR_DATA         , 0                               , 0x3D    },
	{DISPCTRL_WR_DATA         , 0                               , 0x2E    },
	
	{DISPCTRL_WR_CMND         , 0xC7                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },

	{DISPCTRL_WR_CMND         , 0xE8                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x85    },
	{DISPCTRL_WR_DATA		  , 0								, 0x00	  },
	{DISPCTRL_WR_DATA		  , 0								, 0x79	  },

	//Initializing sequence
	{DISPCTRL_WR_CMND         , 0x35                            , 0       },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	
	{DISPCTRL_WR_CMND         , 0x36                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0xD8    },
	
	{DISPCTRL_WR_CMND         , 0x3A                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x55    }, //?
	
	{DISPCTRL_WR_CMND         , 0xB1                            , 0       },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x15    },
	  
	{DISPCTRL_WR_CMND         , 0xB5                            , 0       },
	{DISPCTRL_WR_DATA         , 0                               , 0x04    },
	{DISPCTRL_WR_DATA         , 0                               , 0x04    },
	{DISPCTRL_WR_DATA         , 0                               , 0x0A    },
	{DISPCTRL_WR_DATA         , 0                               , 0x14    },
	
	{DISPCTRL_WR_CMND         , 0xB6                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x0A    },
	{DISPCTRL_WR_DATA         , 0                               , 0xA2    },
	
	{DISPCTRL_WR_CMND         , 0xF6                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x01    },
	{DISPCTRL_WR_DATA         , 0                               , 0x33    },
	
	//Gama setting
	{DISPCTRL_WR_CMND         , 0xF2                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x02    },
	  
	{DISPCTRL_WR_CMND         , 0xE0                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x0E    },
	{DISPCTRL_WR_DATA         , 0                               , 0x2A    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x28    },
	{DISPCTRL_WR_DATA         , 0                               , 0x0B    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x0E    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x06    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x55    },
	{DISPCTRL_WR_DATA         , 0                               , 0x85    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x43    },
	{DISPCTRL_WR_DATA         , 0                               , 0x05    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x0F    },
	{DISPCTRL_WR_DATA         , 0                               , 0x00    },
	{DISPCTRL_WR_DATA         , 0                               , 0x17    },
	{DISPCTRL_WR_DATA         , 0                               , 0x12    },
	{DISPCTRL_WR_DATA         , 0                               , 0x0E    },
	
	{DISPCTRL_WR_CMND         , 0xE1                            , 0       }, 
	{DISPCTRL_WR_DATA         , 0                               , 0x0E    },
	{DISPCTRL_WR_DATA         , 0                               , 0x17    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x1B    },
	{DISPCTRL_WR_DATA         , 0                               , 0x02    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x0F    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x03    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x32    },
	{DISPCTRL_WR_DATA         , 0                               , 0x13    },		
	{DISPCTRL_WR_DATA         , 0                               , 0x47    },
	{DISPCTRL_WR_DATA         , 0                               , 0x03    },
	
	{DISPCTRL_WR_DATA         , 0                               , 0x0E    },
	{DISPCTRL_WR_DATA         , 0                               , 0x0C    },
	{DISPCTRL_WR_DATA         , 0                               , 0x35    },
	{DISPCTRL_WR_DATA         , 0                               , 0x37    },
	{DISPCTRL_WR_DATA         , 0                               , 0x0E    },

	//Sleep out
	{DISPCTRL_WR_CMND         , 0x11                            , 0       },
	
	{DISPCTRL_SLEEP_MS        , 0                               , 120     },
	
	{DISPCTRL_WR_CMND         , 0x29                            , 0       },
	
	{DISPCTRL_SLEEP_MS        , 0                               , 40      },

	//--- END OF COMMAND LIST -----------------------
	{DISPCTRL_LIST_END        , 0                               , 0       }
};

DISPCTRL_REC_T ili9341_panel04_Sleep[] = {
#if 0 //Cori Model
	{DISPCTRL_WR_CMND		  , 0x10							, 0 	  },
	{DISPCTRL_SLEEP_MS		  , 0								, 120	  },
	{DISPCTRL_LIST_END		  , 0								, 0 	  }
#endif

    {DISPCTRL_WR_CMND         , 0x10                            , 0       },
    {DISPCTRL_LIST_END        , 0                               , 0       }
};

