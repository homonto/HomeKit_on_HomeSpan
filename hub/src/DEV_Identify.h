// NOT NEEDED - IN MAIN FILE




//////////////////////////////////
//   DEVICE-SPECIFIC SERVICES   //
//////////////////////////////////

// struct DEV_Identify : Service::AccessoryInformation 
// {

//   int nBlinks;                    // number of times to blink built-in LED in identify routine
//   SpanCharacteristic *identify;   // reference to the Identify Characteristic
  
//   DEV_Identify(const char *name, const char *manu, const char *sn, const char *model, const char *version, int nBlinks) : Service::AccessoryInformation()
//   {
    
//     new Characteristic::Name(name);                   // create all the required Characteristics with values set based on above arguments
//     new Characteristic::Manufacturer(manu);
//     new Characteristic::SerialNumber(sn);    
//     new Characteristic::Model(model);
//     new Characteristic::FirmwareRevision(version);
//     identify=new Characteristic::Identify();          // store a reference to the Identify Characteristic for use below

//     this->nBlinks=nBlinks;                            // store the number of times to blink the LED

//     // pinMode(homeSpan.getStatusPin(),OUTPUT);          // make sure LED is set for output
//   }

//   boolean update()
//   { 
//     #ifdef ERROR_RED_LED_GPIO    
//         LOG0("[%s]: hub called to identify, blinking ERROR LED...\n",__func__);
//         for(int i=0;i<nBlinks;i++)
//         {
//             digitalWrite(ERROR_RED_LED_GPIO,LOW);
//             delay(50);
//             digitalWrite(ERROR_RED_LED_GPIO,HIGH);
//             delay(50);
//         }
//     #elif defined(STATUS_LED_GPIO)
//         LOG0("[%s]: hub called to identify, blinking status LED...",__func__);
//         for(int i=0;i<nBlinks;i++)
//         {
//             digitalWrite(STATUS_LED_GPIO,LOW);
//             delay(50);
//             digitalWrite(STATUS_LED_GPIO,HIGH);
//             delay(50);
//         }
//     #else 
//         LOG0("[%s]: no LED defined - failed\n",__func__);
//         return false;
//     #endif
//     LOG0("[%s]: DONE\n",__func__);
//     return true;
//   }

// };
