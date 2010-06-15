#define ICP_MIO_NAME                  "ICP-MULTI"

#define ICP_MIO_OPEN_ERR              "bad minor nr"

#define ICP_MIO_ALREADY_INIT          "Already initialized"
#define ICP_MIO_LIST_ALREADY_INIT     "MIO list already initialized"
#define ICP_MIO_CANT_INIT             "Can't initialize MIO-Card"
#define ICP_MIO_NOT_FOUND             "No Mio card found"
#define ICP_MIO_NOT_INITIALIZED       "the mio module is not initialized, first start icpMioInit()"
#define ICP_MIO_BAD_CARDNR            "bad id for card: %i (possible: 0..%u)", number, (nrOfMios-1)
#define ICP_MIO_CANT_CANT_GET_POINTER "memory error: can't get pointer to the given element"
#define ICP_MIO_CANT_REG_CHRDEV       "can't register char device for %s", pFs->devName
#define ICP_MIO_PCI_INIT_ERR          "pci initialization error"
#define ICP_MIO_EEPROM_INIT_ERR       "eeprom initialization error"
#define ICP_MIO_AI_INIT_ERR           "ananlog input initialization error"
#define ICP_MIO_AO_INIT_ERR           "ananlog output initialization error"
#define ICP_MIO_DI_INIT_ERR           "digital input initialization error"
#define ICP_MIO_DO_INIT_ERR           "digital output initialization error"
#define ICP_MIO_CTR_INIT_ERR          "counter initialization error"
#define ICP_MIO_IRQ_INIT_ERR          "Interrupt initialization error"

#define ICP_MIO_FS_INIT_ERR           "Linux file system init error"
#define ICP_MIO_FS_CLEANUP_ERR        "Linux file system cleanup error"
#define ICP_MIO_FS_OPEN_ERR1          "Can't open file, not registered"
#define ICP_MIO_FS_OPEN_ERR2          "Can't open file, already opened"
#define ICP_MIO_FS_CLOSE_ERR1         "can't close file"
#define ICP_MIO_FS_CLOSE_ERR2         "can't close file 2"

#define ICP_MIO_AI_CLEANUP_ERR        "ananlog input cleanup err"
#define ICP_MIO_AI_IRQ_INIT_ERR       "analog input irq registration error"
#define ICP_MIO_AI_OPEN_ERR           "analog input open error, bad ch nr param"

#define ICP_MIO_FATAL_ERROR           "Fatal error at %s in line %i", __FILE__, __LINE__

#define ICP_MIO_AI_CH_OPEN_ERR1       "Can't open analog in channel, not initialized"
#define ICP_MIO_AI_CH_OPEN_ERR2       "Can't open file, already opened"

#define ICP_MIO_AO_CLEANUP_ERR        "ananlog output cleanup err"
#define ICP_MIO_AO_CH_OPEN_ERR1       "Can't open analog out channel, not initialized"
#define ICP_MIO_AO_CH_OPEN_ERR2       "Can't open file, already opened"
#define ICP_MIO_AO_IRQ_INIT_ERR       "analog output irq registration error"
#define ICP_MIO_AO_OPEN_ERR           "analog output open error, bad ch nr param"

#define ICP_MIO_DO_IRQ_INIT_ERR       "digital output irq registration error"
#define ICP_MIO_DO_CLEANUP_ERR        "digital output cleanup err"
#define ICP_MIO_DO_OPEN_ERR           "digital output open error, bad ch nr param"
#define ICP_MIO_DO_OPEN_ERR1          "Can't open digital output, not initialized"
#define ICP_MIO_DO_OPEN_ERR2          "Can't open file, already opened"

#define ICP_MIO_DI_IRQ_INIT_ERR       "digital input irq registration error"
#define ICP_MIO_DI_CLEANUP_ERR        "digital input cleanup err"
#define ICP_MIO_DI_OPEN_ERR           "digital input open error, bad ch nr param"
#define ICP_MIO_DI_OPEN_ERR1          "Can't open digital input, not initialized"
#define ICP_MIO_DI_OPEN_ERR2          "Can't open file, already opened"


#define ICP_MIO_CTR_CH_INIT_ERR1      "bad ch number for init"
#define ICP_MIO_CTR_CH_IRQ_INIT_ERR   "counter channel irq registration error"
#define ICP_MIO_CTR_CH_CLEANUP_ERR    "counter channel cleanup err"
#define ICP_MIO_CTR_OPEN_ERR          "counter open error, bad ch nr param"
#define ICP_MIO_CTR_CH_OPEN_ERR1      "Can't open counter channel, not initialized"
#define ICP_MIO_CTR_CH_OPEN_ERR2      "Can't open counter channel, already opened"

#define ICP_GEN_IRQ_REG_ERR           "general irq registration error"

#define ICP_MIO_EEP_READ_ERROR        "can't read eeprom"