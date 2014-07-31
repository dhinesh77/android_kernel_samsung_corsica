

DISPCTRL_REC_T ili9341_panel01_Init[] = {
     /* + Sleep out */
     {DISPCTRL_WR_CMND         , 0x11                           , 0       },   
     {DISPCTRL_SLEEP_MS        , 0                              , 120      },
     /* - Sleep out */


     /* + Initializing Sequnece */
    {DISPCTRL_WR_CMND_DATA    , 0xB4                        , 0x02    },   

    {DISPCTRL_WR_CMND         , 0xB6                           , 0       },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0A    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0xC2    },  /* A2 --> C2 flip the display*/
    {DISPCTRL_WR_DATA         , 0x00                            , 0x27    },
	
    {DISPCTRL_WR_CMND_DATA    , 0xB7                        , 0x06    },   
	
    {DISPCTRL_WR_CMND         , 0xF6                            , 0x0     },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x01    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x30    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },

    {DISPCTRL_WR_CMND_DATA    , 0x3A                        , 0x66    }, /* RGB565 : 0x06 RGB666:0x06 */
    {DISPCTRL_WR_CMND_DATA    , 0x36                        , 0x08    },
    {DISPCTRL_WR_CMND_DATA    , 0x35                        , 0x00    },

	{DISPCTRL_WR_CMND         , 0xCF                            , 0        },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0xFB    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0xFF    },    

    {DISPCTRL_WR_CMND         , 0xED                            , 0        },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x64    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x03    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x12    },  
    {DISPCTRL_WR_DATA         , 0x00                            , 0x81    },  

    {DISPCTRL_WR_CMND         , 0xE8                            , 0        },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x85    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x60    },  

    {DISPCTRL_WR_CMND         , 0xEA                           , 0         },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },
	
    {DISPCTRL_WR_CMND         , 0x2A                           , 0       },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },  
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0xEF    },

    {DISPCTRL_WR_CMND         , 0x2B                           , 0       },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },  
    {DISPCTRL_WR_DATA         , 0x00                            , 0x01    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x3F    },
	
    {DISPCTRL_WR_CMND_DATA    , 0xF2                        , 0x02    },            
    /* - Initializing Sequnece */


    /* + Power Setting Sequence */
    {DISPCTRL_WR_CMND_DATA    , 0xC0                        , 0x18    },
    {DISPCTRL_WR_CMND_DATA    , 0xC1                        , 0x10    },

    {DISPCTRL_WR_CMND         , 0xC5                            , 0x0    },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x2D   },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x4D    }, 
	
    {DISPCTRL_WR_CMND_DATA    , 0xC7                        , 0x00    },
    {DISPCTRL_WR_CMND_DATA    , 0xF7                        , 0x20    },
	
    {DISPCTRL_WR_CMND         , 0xB1                            , 0x0    },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00   },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x1A    }, 
	
    {DISPCTRL_WR_CMND         , 0xCB                            , 0       },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x39    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x2C    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },  
    {DISPCTRL_WR_DATA         , 0x00                            , 0x34    }, 
    {DISPCTRL_WR_DATA         , 0x00                            , 0x02    },  
    /* - Power Setting Sequence */
		
    /* + Gama setting */
    {DISPCTRL_WR_CMND         , 0xE0                            , 0x0     },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0F    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x11    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x10    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x09    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0F    },

    {DISPCTRL_WR_DATA         , 0x00                            , 0x06    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x41    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x84    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x32    },	
    {DISPCTRL_WR_DATA         , 0x00                            , 0x06    },
            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0D    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x04    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0E    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x10    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0C    },
	
    {DISPCTRL_WR_CMND         , 0xE1                            , 0x0     },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x10    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x14    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x02    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x10    },

    {DISPCTRL_WR_DATA         , 0x00                            , 0x04    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x2C    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x22    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x3F    },	
    {DISPCTRL_WR_DATA         , 0x00                            , 0x04    }, 
         
    {DISPCTRL_WR_DATA         , 0x00                            , 0x10    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0A    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x28    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x2E    },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x03    },
    /* - Gama setting */

    {DISPCTRL_WR_CMND,          0x44,                             0x0     },
    {DISPCTRL_WR_DATA,          0x00,                             0x01    },
    {DISPCTRL_WR_DATA,          0x00,                             0x20    },
	
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

     /* Initializing sequence */
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
		
    {DISPCTRL_WR_CMND         , 0xEA                            , 0        },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0xC3    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00    },            

    {DISPCTRL_WR_CMND_DATA    , 0xC0                        , 0x22    },
    {DISPCTRL_WR_CMND_DATA    , 0xC1                        , 0x10    },

    {DISPCTRL_WR_CMND         , 0xC5                            , 0        },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x38    },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x40    },  

    {DISPCTRL_WR_CMND_DATA    , 0xC7                        , 0x00    },   
	
    {DISPCTRL_WR_CMND_DATA    , 0x35                        , 0x00     },
    {DISPCTRL_WR_CMND_DATA    , 0x36                        , 0x08     },
    {DISPCTRL_WR_CMND_DATA    , 0x3A                        , 0x06     },
    {DISPCTRL_WR_CMND_DATA    , 0x26                        , 0x01    },	//RGB565, RGB666:0x06h

    {DISPCTRL_WR_CMND         , 0xB1                            , 0x0      },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x18     },  /* Frame Rtate 0x17 == 83Hz */
	
    {DISPCTRL_WR_CMND         , 0xB4                            , 0x00       },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x02     },            
    
    {DISPCTRL_WR_CMND         , 0xB6                           , 0x0       },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0A     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0xC2     },  

    {DISPCTRL_WR_CMND         , 0xF6                            , 0x0      },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x01     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x30     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00     },

    {DISPCTRL_WR_CMND_DATA    , 0xF2                        	, 0x02     },

     /*Gama setting sequence */
    {DISPCTRL_WR_CMND         , 0xE0                            , 0x0      },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0F     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x1D     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x1A     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0B     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0E     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x06     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x49     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x76     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x39     },	
    {DISPCTRL_WR_DATA         , 0x00                            , 0x08     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x13     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x06     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0B     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x03     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00     },
	
    {DISPCTRL_WR_CMND         , 0xE1                            , 0x0      },   
    {DISPCTRL_WR_DATA         , 0x00                            , 0x00     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x1D     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x21     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x03     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0F     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x03     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x35     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x24     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x46     },	
    {DISPCTRL_WR_DATA         , 0x00                            , 0x04     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0C     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0B     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x34     },            
    {DISPCTRL_WR_DATA         , 0x00                            , 0x37     },
    {DISPCTRL_WR_DATA         , 0x00                            , 0x0F     },

     /* Sleep out sequence */
    {DISPCTRL_WR_CMND         , 0x11                           , 0          },   
    {DISPCTRL_SLEEP_MS         , 0                                , 120      },
    {DISPCTRL_WR_CMND          , 0x44                           , 0x0      },
    {DISPCTRL_WR_DATA          , 0x00                           , 0x01     },
    {DISPCTRL_WR_DATA          , 0x00                           , 0x20     },
    {DISPCTRL_WR_CMND         , 0x29                           , 0          },   

    /* ------------ END OF COMMAND LIST -------------- */
    {DISPCTRL_LIST_END         , 0                                 , 0          }
};

DISPCTRL_REC_T ili9341_panel02_Sleep[] = {
    {DISPCTRL_SLEEP_MS        , 0                               , 10          },
    {DISPCTRL_LIST_END       , 0                                 , 0           }
};

