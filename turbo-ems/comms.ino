/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/
#include "globals.h"
#include "comms.h"
#include "errors.h"
#include "storage.h"
#include "maths.h"
#include "utils.h"
#include "decoders.h"

/*
  Processes the data on the serial buffer.
  Can be either a new command or a continuation of one that is already in progress:
    * cmdPending = If a command has started but is wairing on further data to complete
    * chunkPending = Specifically for the new receive value method where TS will send a known number of contiguous bytes to be written to a table
*/
void command()
{

    if (cmdPending == false) {
        currentCommand = Serial.read();
    }

    switch (currentCommand)
    {

    case 'A': // send x bytes of realtime values
        sendValues(0, SERIAL_PACKET_SIZE, 0x30, 0);   //send values to serial0
        break;


    case 'B': // Burn current values to eeprom
        writeAllConfig();
        break;

    case 'b': // New EEPROM burn command to only burn a single page at a time
        cmdPending = true;

        if (Serial.available() >= 2)
        {
            Serial.read(); //Ignore the first table value, it's always 0
            writeConfig(Serial.read());
            cmdPending = false;
        }
        break;

    case 'C': // test communications. This is used by Tunerstudio to see whether there is an ECU on a given serial port
        testComm();
        break;

    case 'c': //Send the current loops/sec value
        Serial.write(lowByte(currentStatus.loopsPerSecond));
        Serial.write(highByte(currentStatus.loopsPerSecond));
        break;

    case 'd': // Send a CRC32 value of a given page
        cmdPending = true;

        if (Serial.available() >= 2)
        {
            Serial.read(); //Ignore the first table value, it's always 0
            uint32_t CRC32_val = calculateCRC32( Serial.read() );

            //Split the 4 bytes of the CRC32 value into individual bytes and send
            Serial.write( ((CRC32_val >> 24) & 255) );
            Serial.write( ((CRC32_val >> 16) & 255) );
            Serial.write( ((CRC32_val >> 8) & 255) );
            Serial.write( (CRC32_val & 255) );

            cmdPending = false;
        }
        break;

    //The following can be used to show the amount of free memory

    case 'E': // receive command button commands
        cmdPending = true;

        if(Serial.available() >= 2)
        {
            cmdGroup = Serial.read();
            cmdValue = Serial.read();
            cmdCombined = word(cmdGroup, cmdValue);
            if (currentStatus.RPM == 0) {
                commandButtons();
            }

            cmdPending = false;
        }
        break;

    case 'F': // send serial protocol version
        Serial.print("001");
        break;

    case 'H': //Start the tooth logger
//      currentStatus.toothLogEnabled = true;
//      currentStatus.compositeLogEnabled = false; //Safety first (Should never be required)
//      toothHistoryIndex = 0;
//      toothHistorySerialIndex = 0;
//
//      //Disconnect the standard interrupt and add the logger version
//      detachInterrupt( digitalPinToInterrupt(PIN_CRANK) );
//      attachInterrupt( digitalPinToInterrupt(PIN_CRANK), loggerPrimaryISR, CHANGE );
//
//      detachInterrupt( digitalPinToInterrupt(PIN_CAM) );
//      attachInterrupt( digitalPinToInterrupt(PIN_CAM), loggerSecondaryISR, CHANGE );

        Serial.write(1); //TS needs an acknowledgement that this was received. I don't know if this is the correct response, but it seems to work
        break;

    case 'h': //Stop the tooth logger
        break;

    case 'J': //Start the composite logger
        Serial.write(1); //TS needs an acknowledgement that this was received. I don't know if this is the correct response, but it seems to work
        break;

    case 'j': //Stop the composite logger
        break;

    case 'L': // List the contents of current page in human readable form
        sendPage(true);
        break;

    case 'm': //Send the current free memory
        currentStatus.freeRAM = freeRam();
        Serial.write(lowByte(currentStatus.freeRAM));
        Serial.write(highByte(currentStatus.freeRAM));
        break;

    case 'N': // Displays a new line.  Like pushing enter in a text editor
        Serial.println();
        break;

    case 'P': // set the current page
        //A 2nd byte of data is required after the 'P' specifying the new page number.
        cmdPending = true;

        if (Serial.available() > 0)
        {
            currentPage = Serial.read();
            //This converts the ascii number char into binary. Note that this will break everyything if there are ever more than 48 pages (48 = asci code for '0')
            if (currentPage >= '0') {
                currentPage -= '0';
            }
            // Detecting if the current page is a table/map
            if ( (currentPage == veMapPage) || (currentPage == ignMapPage) || (currentPage == afrMapPage) ) {
                isMap = true;
            }
            else {
                isMap = false;
            }
            cmdPending = false;
        }
        break;

    /*
    * New method for sending page values
    */
    case 'p':
        cmdPending = true;

        //6 bytes required:
        //2 - Page identifier
        //2 - offset
        //2 - Length
        if(Serial.available() >= 6)
        {
            byte offset1, offset2, length1, length2;
            int length;
            byte tempPage;

            Serial.read(); // First byte of the page identifier can be ignored. It's always 0
            tempPage = Serial.read();
            //currentPage = 1;
            offset1 = Serial.read();
            offset2 = Serial.read();
            valueOffset = word(offset2, offset1);
            length1 = Serial.read();
            length2 = Serial.read();
            length = word(length2, length1);
            for(int i = 0; i < length; i++)
            {
                Serial.write( getPageValue(tempPage, valueOffset + i) );
            }

            cmdPending = false;
        }
        break;

    case 'Q': // send code version
        Serial.print(F("speeduino 201903-dev"));
        break;

    case 'r': //New format for the optimised OutputChannels
        cmdPending = true;
        byte cmd;
        if (Serial.available() >= 6)
        {
            tsCanId = Serial.read(); //Read the $tsCanId
            cmd = Serial.read(); // read the command

            uint16_t offset, length;
            if(cmd == 0x30) //Send output channels command 0x30 is 48dec
            {
                byte tmp;
                tmp = Serial.read();
                offset = word(Serial.read(), tmp);
                tmp = Serial.read();
                length = word(Serial.read(), tmp);
                sendValues(offset, length,cmd, 0);
            }
            else
            {
                //No other r/ commands should be called
            }
            cmdPending = false;
        }
        break;

    case 'S': // send code version
        Serial.print(F("Speeduino 2019.03-dev"));
        currentStatus.secl = 0; //This is required in TS3 due to its stricter timings
        break;

    case 'T': //Send 256 tooth log entries to Tuner Studios tooth logger
        break;

    case 't': // receive new Calibration info. Command structure: "t", <tble_idx> <data array>. This is an MS2/Extra command, NOT part of MS1 spec
        byte tableID;
        //byte canID;

        //The first 2 bytes sent represent the canID and tableID
        while (Serial.available() == 0) { }
        tableID = Serial.read(); //Not currently used for anything

        receiveCalibration(tableID); //Receive new values and store in memory

        break;

    case 'U': //User wants to reset the Arduino (probably for FW update)
        break;

    case 'V': // send VE table and constants in binary
        sendPage(false);
        break;

    case 'W': // receive new VE obr constant at 'W'+<offset>+<newbyte>
        cmdPending = true;

        if (isMap)
        {
            if(Serial.available() >= 3) // 1 additional byte is required on the MAP pages which are larger than 255 bytes
            {
                byte offset1, offset2;
                offset1 = Serial.read();
                offset2 = Serial.read();
                valueOffset = word(offset2, offset1);
                receiveValue(valueOffset, Serial.read());
                cmdPending = false;
            }
        }
        else
        {
            if(Serial.available() >= 2)
            {
                valueOffset = Serial.read();
                receiveValue(valueOffset, Serial.read());
                cmdPending = false;
            }
        }

        break;

    case 'w':
        cmdPending = true;

        if(chunkPending == false)
        {
            //This means it's a new request
            //7 bytes required:
            //2 - Page identifier
            //2 - offset
            //2 - Length
            //1 - 1st New value
            if(Serial.available() >= 7)
            {
                byte offset1, offset2, length1, length2;

                Serial.read(); // First byte of the page identifier can be ignored. It's always 0
                currentPage = Serial.read();
                //currentPage = 1;
                offset1 = Serial.read();
                offset2 = Serial.read();
                valueOffset = word(offset2, offset1);
                length1 = Serial.read(); // Length to be written (Should always be 1)
                length2 = Serial.read(); // Length to be written (Should always be 1)
                chunkSize = word(length2, length1);

                chunkPending = true;
                chunkComplete = 0;
            }
        }
        //This CANNOT be an else of the above if statement as chunkPending gets set to true above
        if(chunkPending == true)
        {
            while( (Serial.available() > 0) && (chunkComplete < chunkSize) )
            {
                receiveValue( (valueOffset + chunkComplete), Serial.read());
                chunkComplete++;
            }
            if(chunkComplete >= chunkSize) {
                cmdPending = false;
                chunkPending = false;
            }
        }
        break;

    case 'Z': //Totally non-standard testing function.
        Serial.print("req_fuel_uS:");
        Serial.println(req_fuel_uS);

        Serial.print("inj_opentime_uS:");
        Serial.println(inj_opentime_uS);

        Serial.flush();
        break;

    case 'z': //Send 256 tooth log entries to a terminal emulator
        break;

    case '`': //Custom 16u2 firmware is making its presence known
        cmdPending = true;

        if (Serial.available() >= 1) {
            configPage4.bootloaderCaps = Serial.read();
            cmdPending = false;
        }
        break;


    case '?':
        Serial.println
        (F(
             "\n"
             "===Command Help===\n\n"
             "All commands are single character and are concatenated with their parameters \n"
             "without spaces."
             "Syntax:  <command>+<parameter1>+<parameter2>+<parameterN>\n\n"
             "===List of Commands===\n\n"
             "A - Displays 31 bytes of currentStatus values in binary (live data)\n"
             "B - Burn current map and configPage values to eeprom\n"
             "C - Test COM port.  Used by Tunerstudio to see whether an ECU is on a given serial \n"
             "    port. Returns a binary number.\n"
             "N - Print new line.\n"
             "P - Set current page.  Syntax:  P+<pageNumber>\n"
             "R - Same as A command\n"
             "S - Display signature number\n"
             "Q - Same as S command\n"
             "V - Display map or configPage values in binary\n"
             "W - Set one byte in map or configPage.  Expects binary parameters. \n"
             "    Syntax:  W+<offset>+<newbyte>\n"
             "t - Set calibration values.  Expects binary parameters.  Table index is either 0, \n"
             "    1, or 2.  Syntax:  t+<tble_idx>+<newValue1>+<newValue2>+<newValueN>\n"
             "Z - Display calibration values\n"
             "T - Displays 256 tooth log entries in binary\n"
             "r - Displays 256 tooth log entries\n"
             "U - Prepare for firmware update. The next byte received will cause the Arduino to reset.\n"
             "? - Displays this help page"
         ));

        break;

    default:
        break;
    }
}

/*
This function returns the current values of a fixed group of variables
*/
//void sendValues(int packetlength, byte portNum)
void sendValues(uint16_t offset, uint16_t packetLength, byte cmd, byte portNum)
{
    byte fullStatus[SERIAL_PACKET_SIZE];

    if (portNum != 3)
    {
        if(requestCount == 0) {
            currentStatus.secl = 0;
        }
        requestCount++;
    }

    currentStatus.spark ^= (-currentStatus.hasSync ^ currentStatus.spark) & (1 << BIT_SPARK_SYNC); //Set the sync bit of the Spark variable to match the hasSync variable

    fullStatus[0] = currentStatus.secl; //secl is simply a counter that increments each second. Used to track unexpected resets (Which will reset this count to 0)
    fullStatus[1] = currentStatus.status1; //status1 Bitfield
    fullStatus[2] = currentStatus.engine; //Engine Status Bitfield
    fullStatus[3] = currentStatus.syncLossCounter;
    fullStatus[4] = lowByte(currentStatus.MAP); //2 bytes for MAP
    fullStatus[5] = highByte(currentStatus.MAP);
    fullStatus[6] = (byte)(currentStatus.IAT + CALIBRATION_TEMPERATURE_OFFSET); //mat
    fullStatus[7] = (byte)(currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //Coolant ADC
    fullStatus[8] = currentStatus.batCorrection; //Battery voltage correction (%)
    fullStatus[9] = currentStatus.battery10; //battery voltage
    fullStatus[10] = currentStatus.O2; //O2
    fullStatus[11] = currentStatus.egoCorrection; //Exhaust gas correction (%)
    fullStatus[12] = currentStatus.iatCorrection; //Air temperature Correction (%)
    fullStatus[13] = currentStatus.wueCorrection; //Warmup enrichment (%)
    fullStatus[14] = lowByte(currentStatus.RPM); //rpm HB
    fullStatus[15] = highByte(currentStatus.RPM); //rpm LB
    fullStatus[16] = (byte)(currentStatus.TAEamount >> 1); //TPS acceleration enrichment (%) divided by 2 (Can exceed 255)
    fullStatus[17] = currentStatus.corrections; //Total GammaE (%)
    fullStatus[18] = currentStatus.VE; //Current VE 1 (%)
    fullStatus[19] = currentStatus.afrTarget;
    fullStatus[20] = lowByte(currentStatus.PW1); //Pulsewidth 1 multiplied by 10 in ms. Have to convert from uS to mS.
    fullStatus[21] = highByte(currentStatus.PW1); //Pulsewidth 1 multiplied by 10 in ms. Have to convert from uS to mS.
    fullStatus[22] = currentStatus.tpsDOT; //TPS DOT
    fullStatus[23] = currentStatus.advance;
    fullStatus[24] = currentStatus.TPS; // TPS (0% to 100%)
    //Need to split the int loopsPerSecond value into 2 bytes
    fullStatus[25] = lowByte(currentStatus.loopsPerSecond);
    fullStatus[26] = highByte(currentStatus.loopsPerSecond);

    //The following can be used to show the amount of free memory
    currentStatus.freeRAM = freeRam();
    fullStatus[27] = lowByte(currentStatus.freeRAM); //(byte)((currentStatus.loopsPerSecond >> 8) & 0xFF);
    fullStatus[28] = highByte(currentStatus.freeRAM);

    fullStatus[29] = (byte)(currentStatus.boostTarget >> 1); //Divide boost target by 2 to fit in a byte
    fullStatus[30] = (byte)(currentStatus.boostDuty / 100);
    fullStatus[31] = currentStatus.spark; //Spark related bitfield

    //rpmDOT must be sent as a signed integer
    fullStatus[32] = lowByte(currentStatus.rpmDOT);
    fullStatus[33] = highByte(currentStatus.rpmDOT);

    fullStatus[34] = currentStatus.ethanolPct; //Flex sensor value (or 0 if not used)
    fullStatus[35] = currentStatus.flexCorrection; //Flex fuel correction (% above or below 100)
    fullStatus[36] = currentStatus.flexIgnCorrection; //Ignition correction (Increased degrees of advance) for flex fuel

    fullStatus[37] = currentStatus.idleLoad;
    fullStatus[38] = currentStatus.testOutputs;

    fullStatus[39] = currentStatus.O2_2; //O2
    fullStatus[40] = currentStatus.baro; //Barometer value

    fullStatus[41] = lowByte(currentStatus.canin[0]);
    fullStatus[42] = highByte(currentStatus.canin[0]);
    fullStatus[43] = lowByte(currentStatus.canin[1]);
    fullStatus[44] = highByte(currentStatus.canin[1]);
    fullStatus[45] = lowByte(currentStatus.canin[2]);
    fullStatus[46] = highByte(currentStatus.canin[2]);
    fullStatus[47] = lowByte(currentStatus.canin[3]);
    fullStatus[48] = highByte(currentStatus.canin[3]);
    fullStatus[49] = lowByte(currentStatus.canin[4]);
    fullStatus[50] = highByte(currentStatus.canin[4]);
    fullStatus[51] = lowByte(currentStatus.canin[5]);
    fullStatus[52] = highByte(currentStatus.canin[5]);
    fullStatus[53] = lowByte(currentStatus.canin[6]);
    fullStatus[54] = highByte(currentStatus.canin[6]);
    fullStatus[55] = lowByte(currentStatus.canin[7]);
    fullStatus[56] = highByte(currentStatus.canin[7]);
    fullStatus[57] = lowByte(currentStatus.canin[8]);
    fullStatus[58] = highByte(currentStatus.canin[8]);
    fullStatus[59] = lowByte(currentStatus.canin[9]);
    fullStatus[60] = highByte(currentStatus.canin[9]);
    fullStatus[61] = lowByte(currentStatus.canin[10]);
    fullStatus[62] = highByte(currentStatus.canin[10]);
    fullStatus[63] = lowByte(currentStatus.canin[11]);
    fullStatus[64] = highByte(currentStatus.canin[11]);
    fullStatus[65] = lowByte(currentStatus.canin[12]);
    fullStatus[66] = highByte(currentStatus.canin[12]);
    fullStatus[67] = lowByte(currentStatus.canin[13]);
    fullStatus[68] = highByte(currentStatus.canin[13]);
    fullStatus[69] = lowByte(currentStatus.canin[14]);
    fullStatus[70] = highByte(currentStatus.canin[14]);
    fullStatus[71] = lowByte(currentStatus.canin[15]);
    fullStatus[72] = highByte(currentStatus.canin[15]);

    fullStatus[73] = currentStatus.tpsADC;
    fullStatus[74] = getNextError();

    fullStatus[75] = lowByte(currentStatus.PW2); //Pulsewidth 2 multiplied by 10 in ms. Have to convert from uS to mS.
    fullStatus[76] = highByte(currentStatus.PW2); //Pulsewidth 2 multiplied by 10 in ms. Have to convert from uS to mS.
    fullStatus[77] = lowByte(currentStatus.PW3); //Pulsewidth 3 multiplied by 10 in ms. Have to convert from uS to mS.
    fullStatus[78] = highByte(currentStatus.PW3); //Pulsewidth 3 multiplied by 10 in ms. Have to convert from uS to mS.
    fullStatus[79] = lowByte(currentStatus.PW4); //Pulsewidth 4 multiplied by 10 in ms. Have to convert from uS to mS.
    fullStatus[80] = highByte(currentStatus.PW4); //Pulsewidth 4 multiplied by 10 in ms. Have to convert from uS to mS.

    fullStatus[81] = currentStatus.status3;
    fullStatus[82] = lowByte(currentStatus.flexBoostCorrection);
    fullStatus[83] = highByte(currentStatus.flexBoostCorrection);

    fullStatus[84] = currentStatus.nChannels;
    fullStatus[85] = lowByte(currentStatus.fuelLoad);
    fullStatus[86] = highByte(currentStatus.fuelLoad);
    fullStatus[87] = lowByte(currentStatus.ignLoad);
    fullStatus[88] = highByte(currentStatus.ignLoad);
    fullStatus[89] = lowByte(currentStatus.dwell);
    fullStatus[90] = highByte(currentStatus.dwell);

    for(byte x=0; x<packetLength; x++)
    {
        if (portNum == 0) {
            Serial.write(fullStatus[offset+x]);
        }
    }

}

void receiveValue(uint16_t valueOffset, byte newValue)
{

    void* pnt_configPage;//This only stores the address of the value that it's pointing to and not the max size
    int tempOffset;

    switch (currentPage)
    {
    case veMapPage:
        if (valueOffset < 256) //New value is part of the fuel map
        {
            fuelTable.values[15 - (valueOffset / 16)][valueOffset % 16] = newValue;
        }
        else
        {
            //Check whether this is on the X (RPM) or Y (MAP/TPS) axis
            if (valueOffset < 272)
            {
                //X Axis
                fuelTable.axisX[(valueOffset - 256)] = ((int)(newValue) * TABLE_RPM_MULTIPLIER); //The RPM values sent by megasquirt are divided by 100, need to multiple it back by 100 to make it correct (TABLE_RPM_MULTIPLIER)
            }
            else if(valueOffset < 288)
            {
                //Y Axis
                tempOffset = 15 - (valueOffset - 272); //Need to do a translation to flip the order (Due to us using (0,0) in the top left rather than bottom right
                fuelTable.axisY[tempOffset] = (int)(newValue) * TABLE_LOAD_MULTIPLIER;
            }
            else
            {
                //This should never happen. It means there's an invalid offset value coming through
            }
        }
        break;

    case veSetPage:
        pnt_configPage = &configPage2; //Setup a pointer to the relevant config page
        //For some reason, TunerStudio is sending offsets greater than the maximum page size. I'm not sure if it's their bug or mine, but the fix is to only update the config page if the offset is less than the maximum size
        if (valueOffset < npage_size[veSetPage])
        {
            *((byte *)pnt_configPage + (byte)valueOffset) = newValue;
        }
        break;

    case ignMapPage: //Ignition settings page (Page 2)
        if (valueOffset < 256) //New value is part of the ignition map
        {
            ignitionTable.values[15 - (valueOffset / 16)][valueOffset % 16] = newValue;
        }
        else
        {
            //Check whether this is on the X (RPM) or Y (MAP/TPS) axis
            if (valueOffset < 272)
            {
                //X Axis
                ignitionTable.axisX[(valueOffset - 256)] = (int)(newValue) * TABLE_RPM_MULTIPLIER; //The RPM values sent by megasquirt are divided by 100, need to multiple it back by 100 to make it correct
            }
            else if(valueOffset < 288)
            {
                //Y Axis
                tempOffset = 15 - (valueOffset - 272); //Need to do a translation to flip the order
                ignitionTable.axisY[tempOffset] = (int)(newValue) * TABLE_LOAD_MULTIPLIER;
            }
        }
        break;

    case ignSetPage:
        pnt_configPage = &configPage4;
        //For some reason, TunerStudio is sending offsets greater than the maximum page size. I'm not sure if it's their bug or mine, but the fix is to only update the config page if the offset is less than the maximum size
        if (valueOffset < npage_size[ignSetPage])
        {
            *((byte *)pnt_configPage + (byte)valueOffset) = newValue;
        }
        break;

    case afrMapPage: //Air/Fuel ratio target settings page
        if (valueOffset < 256) //New value is part of the afr map
        {
            afrTable.values[15 - (valueOffset / 16)][valueOffset % 16] = newValue;
        }
        else
        {
            //Check whether this is on the X (RPM) or Y (MAP/TPS) axis
            if (valueOffset < 272)
            {
                //X Axis
                afrTable.axisX[(valueOffset - 256)] = int(newValue) * TABLE_RPM_MULTIPLIER; //The RPM values sent by megasquirt are divided by 100, need to multiply it back by 100 to make it correct (TABLE_RPM_MULTIPLIER)
            }
            else
            {
                //Y Axis
                tempOffset = 15 - (valueOffset - 272); //Need to do a translation to flip the order
                afrTable.axisY[tempOffset] = int(newValue) * TABLE_LOAD_MULTIPLIER;

            }
        }
        break;

    case afrSetPage:
        pnt_configPage = &configPage6;
        //For some reason, TunerStudio is sending offsets greater than the maximum page size. I'm not sure if it's their bug or mine, but the fix is to only update the config page if the offset is less than the maximum size
        if (valueOffset < npage_size[afrSetPage])
        {
            *((byte *)pnt_configPage + (byte)valueOffset) = newValue;
        }
        break;

    case boostvvtPage: //Boost, VVT and staging maps (all 8x8)
        if (valueOffset < 64) //New value is part of the boost map
        {
            boostTable.values[7 - (valueOffset / 8)][valueOffset % 8] = newValue;
        }
        else if (valueOffset < 72) //New value is on the X (RPM) axis of the boost table
        {
            boostTable.axisX[(valueOffset - 64)] = int(newValue) * TABLE_RPM_MULTIPLIER; //The RPM values sent by TunerStudio are divided by 100, need to multiply it back by 100 to make it correct (TABLE_RPM_MULTIPLIER)
        }
        else if (valueOffset < 80) //New value is on the Y (TPS) axis of the boost table
        {
            boostTable.axisY[(7 - (valueOffset - 72))] = int(newValue); //TABLE_LOAD_MULTIPLIER is NOT used for boost as it is TPS based (0-100)
        }
        //End of boost table
        else if (valueOffset < 144) //New value is part of the vvt map
        {
            tempOffset = valueOffset - 80;
            vvtTable.values[7 - (tempOffset / 8)][tempOffset % 8] = newValue;
        }
        else if (valueOffset < 152) //New value is on the X (RPM) axis of the vvt table
        {
            tempOffset = valueOffset - 144;
            vvtTable.axisX[tempOffset] = int(newValue) * TABLE_RPM_MULTIPLIER; //The RPM values sent by TunerStudio are divided by 100, need to multiply it back by 100 to make it correct (TABLE_RPM_MULTIPLIER)
        }
        else if (valueOffset < 160) //New value is on the Y (Load) axis of the vvt table
        {
            tempOffset = valueOffset - 152;
            vvtTable.axisY[(7 - tempOffset)] = int(newValue); //TABLE_LOAD_MULTIPLIER is NOT used for vvt as it is TPS based (0-100)
        }
        //End of vvt table
        else if (valueOffset < 224) //New value is part of the staging map
        {
            tempOffset = valueOffset - 160;
            stagingTable.values[7 - (tempOffset / 8)][tempOffset % 8] = newValue;
        }
        else if (valueOffset < 232) //New value is on the X (RPM) axis of the staging table
        {
            tempOffset = valueOffset - 224;
            stagingTable.axisX[tempOffset] = int(newValue) * TABLE_RPM_MULTIPLIER; //The RPM values sent by TunerStudio are divided by 100, need to multiply it back by 100 to make it correct (TABLE_RPM_MULTIPLIER)
        }
        else if (valueOffset < 240) //New value is on the Y (Load) axis of the staging table
        {
            tempOffset = valueOffset - 232;
            stagingTable.axisY[(7 - tempOffset)] = int(newValue) * TABLE_LOAD_MULTIPLIER;
        }
        break;

    case seqFuelPage:
        if (valueOffset < 36) {
            trim1Table.values[5 - (valueOffset / 6)][valueOffset % 6] = newValue;    //Trim1 values
        }
        else if (valueOffset < 42) {
            trim1Table.axisX[(valueOffset - 36)] = int(newValue) * TABLE_RPM_MULTIPLIER;    //New value is on the X (RPM) axis of the trim1 table. The RPM values sent by TunerStudio are divided by 100, need to multiply it back by 100 to make it correct (TABLE_RPM_MULTIPLIER)
        }
        else if (valueOffset < 48) {
            trim1Table.axisY[(5 - (valueOffset - 42))] = int(newValue) * TABLE_LOAD_MULTIPLIER;    //New value is on the Y (TPS) axis of the boost table
        }
        //Trim table 2
        else if (valueOffset < 84) {
            tempOffset = valueOffset - 48;    //New value is part of the trim2 map
            trim2Table.values[5 - (tempOffset / 6)][tempOffset % 6] = newValue;
        }
        else if (valueOffset < 90) {
            tempOffset = valueOffset - 84;    //New value is on the X (RPM) axis of the table. The RPM values sent by TunerStudio are divided by 100, need to multiply it back by 100 to make it correct (TABLE_RPM_MULTIPLIER)
            trim2Table.axisX[tempOffset] = int(newValue) * TABLE_RPM_MULTIPLIER;
        }
        else if (valueOffset < 96) {
            tempOffset = valueOffset - 90;    //New value is on the Y (Load) axis of the table
            trim2Table.axisY[(5 - tempOffset)] = int(newValue) * TABLE_LOAD_MULTIPLIER;
        }
        //Trim table 3
        else if (valueOffset < 132) {
            tempOffset = valueOffset - 96;    //New value is part of the trim2 map
            trim3Table.values[5 - (tempOffset / 6)][tempOffset % 6] = newValue;
        }
        else if (valueOffset < 138) {
            tempOffset = valueOffset - 132;    //New value is on the X (RPM) axis of the table. The RPM values sent by TunerStudio are divided by 100, need to multiply it back by 100 to make it correct (TABLE_RPM_MULTIPLIER)
            trim3Table.axisX[tempOffset] = int(newValue) * TABLE_RPM_MULTIPLIER;
        }
        else if (valueOffset < 144) {
            tempOffset = valueOffset - 138;    //New value is on the Y (Load) axis of the table
            trim3Table.axisY[(5 - tempOffset)] = int(newValue) * TABLE_LOAD_MULTIPLIER;
        }
        //Trim table 4
        else if (valueOffset < 180) {
            tempOffset = valueOffset - 144;    //New value is part of the trim2 map
            trim4Table.values[5 - (tempOffset / 6)][tempOffset % 6] = newValue;
        }
        else if (valueOffset < 186) {
            tempOffset = valueOffset - 180;    //New value is on the X (RPM) axis of the table. The RPM values sent by TunerStudio are divided by 100, need to multiply it back by 100 to make it correct (TABLE_RPM_MULTIPLIER)
            trim4Table.axisX[tempOffset] = int(newValue) * TABLE_RPM_MULTIPLIER;
        }
        else if (valueOffset < 192) {
            tempOffset = valueOffset - 186;    //New value is on the Y (Load) axis of the table
            trim4Table.axisY[(5 - tempOffset)] = int(newValue) * TABLE_LOAD_MULTIPLIER;
        }

        break;

    case canbusPage:
        pnt_configPage = &configPage9;
        //For some reason, TunerStudio is sending offsets greater than the maximum page size. I'm not sure if it's their bug or mine, but the fix is to only update the config page if the offset is less than the maximum size
        if (valueOffset < npage_size[currentPage])
        {
            *((byte *)pnt_configPage + (byte)valueOffset) = newValue;
        }
        break;

    case warmupPage:
        pnt_configPage = &configPage10;
        //For some reason, TunerStudio is sending offsets greater than the maximum page size. I'm not sure if it's their bug or mine, but the fix is to only update the config page if the offset is less than the maximum size
        if (valueOffset < npage_size[currentPage])
        {
            *((byte *)pnt_configPage + (byte)valueOffset) = newValue;
        }
        break;

    default:
        break;
    }
    //if(Serial.available() > 16) { command(); }
}

/*
sendPage() packs the data within the current page (As set with the 'P' command)
into a buffer and sends it.
Note that some translation of the data is required to lay it out in the way Megasqurit / TunerStudio expect it
useChar - If true, all values are send as chars, this is for the serial command line interface. TunerStudio expects data as raw values, so this must be set false in that case
*/
void sendPage(bool useChar)
{
    void* pnt_configPage = &configPage2; //Default value is for safety only. Will be changed below if needed.
    struct table3D currentTable = fuelTable; //Default value is for safety only. Will be changed below if needed.
    byte currentTitleIndex = 0;// This corresponds to the count up to the first char of a string in pageTitles
    bool sendComplete = false; //Used to track whether all send operations are complete

    switch (currentPage)
    {
    case veMapPage:
        currentTitleIndex = 0;
        currentTable = fuelTable;
        break;

    case veSetPage:
        // currentTitleIndex = 27;
        if (useChar)
        {
            uint16_t* pnt16_configPage;
            // To Display Values from Config Page 1
            // When casting to the __FlashStringHelper type Serial.println uses the same subroutine as when using the F macro
            Serial.println((const __FlashStringHelper *)&pageTitles[27]);//27 is the index to the first char in the second sting in pageTitles
            // The following loop displays in human readable form of all byte values in config page 1 up to but not including the first array.
            // incrementing void pointers is cumbersome. Thus we have "pnt_configPage = (byte *)pnt_configPage + 1"
            for (pnt_configPage = &configPage2; pnt_configPage < &configPage2.wueValues[0]; pnt_configPage = (byte *)pnt_configPage + 1) {
                Serial.println(*((byte *)pnt_configPage));
            }
            for (byte x = 10; x; x--)// The x between the ';' has the same representation as the "x != 0" test or comparision
            {
                Serial.print(configPage2.wueValues[10 - x]);// This displays the values horizantially on the screen
                Serial.print(' ');
            }
            Serial.println();
            for (pnt_configPage = (byte *)&configPage2.wueValues[9] + 1; pnt_configPage < &configPage2.inj1Ang; pnt_configPage = (byte *)pnt_configPage + 1) {
                Serial.println(*((byte *)pnt_configPage));// This displays all the byte values between the last array up to but not including the first unsigned int on config page 1
            }
            // The following loop displays four unsigned ints
            for (pnt16_configPage = (uint16_t *)&configPage2.inj1Ang; pnt16_configPage < (uint16_t*)&configPage2.inj4Ang + 1; pnt16_configPage = (uint16_t*)pnt16_configPage + 1)
            {
                Serial.println(*((uint16_t *)pnt16_configPage));
            }
            // Following loop displays byte values between the unsigned ints
            for (pnt_configPage = (uint16_t *)&configPage2.inj4Ang + 1; pnt_configPage < &configPage2.mapMax; pnt_configPage = (byte *)pnt_configPage + 1) {
                Serial.println(*((byte *)pnt_configPage));
            }
            Serial.println(configPage2.mapMax);
            // Following loop displays remaining byte values of the page
            for (pnt_configPage = (uint16_t *)&configPage2.mapMax + 1; pnt_configPage < (byte *)&configPage2 + npage_size[veSetPage]; pnt_configPage = (byte *)pnt_configPage + 1) {
                Serial.println(*((byte *)pnt_configPage));
            }
            sendComplete = true;
        }
        else {
            pnt_configPage = &configPage2;    //Create a pointer to Page 1 in memory
        }
        break;

    case ignMapPage:
        currentTitleIndex = 42;// the index to the first char of the third string in pageTitles
        currentTable = ignitionTable;
        break;

    case ignSetPage:
        //currentTitleIndex = 56;
        if (useChar)
        {
            //To Display Values from Config Page 2
            Serial.println((const __FlashStringHelper *)&pageTitles[56]);
            Serial.println(configPage4.triggerAngle);// configPsge2.triggerAngle is an int so just display it without complication
            // Following loop displays byte values after that first int up to but not including the first array in config page 2
            for (pnt_configPage = (int *)&configPage4 + 1; pnt_configPage < &configPage4.taeBins[0]; pnt_configPage = (byte *)pnt_configPage + 1) {
                Serial.println(*((byte *)pnt_configPage));
            }
            for (byte y = 2; y; y--)// Displaying two equal sized arrays
            {
                byte * currentVar;// A placeholder for each array
                if (y == 2) {
                    currentVar = configPage4.taeBins;
                }
                else {
                    currentVar = configPage4.taeValues;
                }

                for (byte j = 4; j; j--)
                {
                    Serial.print(currentVar[4 - j]);
                    Serial.print(' ');
                }
                Serial.println();
            }
            for (byte x = 10; x ; x--)
            {
                Serial.print(configPage4.wueBins[10 - x]);//Displaying array horizontally across screen
                Serial.print(' ');
            }
            Serial.println();
            Serial.println(configPage4.dwellLimit);// Little lonely byte stuck between two arrays. No complications just display it.
            for (byte x = 6; x; x--)
            {
                Serial.print(configPage4.dwellCorrectionValues[6 - x]);
                Serial.print(' ');
            }
            Serial.println();
            for (pnt_configPage = (byte *)&configPage4.dwellCorrectionValues[5] + 1; pnt_configPage < (byte *)&configPage4 + npage_size[ignSetPage]; pnt_configPage = (byte *)pnt_configPage + 1)
            {
                Serial.println(*((byte *)pnt_configPage));// Displaying remaining byte values of the page
            }
            sendComplete = true;
        }
        else {
            pnt_configPage = &configPage4;    //Create a pointer to Page 2 in memory
        }
        break;

    case afrMapPage:
        currentTitleIndex = 71;//Array index to next string
        currentTable = afrTable;
        break;

    case afrSetPage:
        //currentTitleIndex = 91;
        if (useChar)
        {
            //To Display Values from Config Page 3
            Serial.println((const __FlashStringHelper *)&pageTitles[91]);//special typecasting to enable suroutine that the F macro uses
            for (pnt_configPage = &configPage6; pnt_configPage < &configPage6.voltageCorrectionBins[0]; pnt_configPage = (byte *)pnt_configPage + 1)
            {
                Serial.println(*((byte *)pnt_configPage));// Displaying byte values of config page 3 up to but not including the first array
            }
            for (byte y = 2; y; y--)// Displaying two equally sized arrays that are next to each other
            {
                byte * currentVar;
                if (y == 2) {
                    currentVar = configPage6.voltageCorrectionBins;
                }
                else {
                    currentVar = configPage6.injVoltageCorrectionValues;
                }

                for (byte x = 6; x; x--)
                {
                    Serial.print(currentVar[6 - x]);
                    Serial.print(' ');
                }
                Serial.println();
            }
            for (byte y = 2; y; y--)// and again
            {
                byte* currentVar;
                if (y == 2) {
                    currentVar = configPage6.airDenBins;
                }
                else {
                    currentVar = configPage6.airDenRates;
                }

                for (byte x = 9; x; x--)
                {
                    Serial.print(currentVar[9 - x]);
                    Serial.print(' ');
                }
                Serial.println();
            }
            // Following loop displays the remaining byte values of the page
            for (pnt_configPage = (byte *)&configPage6.airDenRates[8] + 1; pnt_configPage < (byte *)&configPage6 + npage_size[afrSetPage]; pnt_configPage = (byte *)pnt_configPage + 1)
            {
                Serial.println(*((byte *)pnt_configPage));
            }
            sendComplete = true;
        }
        else {
            pnt_configPage = &configPage6;    //Create a pointer to Page 3 in memory
        }

        //Old configPage4 STARTED HERE!
        //currentTitleIndex = 106;
        //To Display Values from Config Page 4
        if (useChar)
        {
            Serial.println((const __FlashStringHelper *)&pageTitles[106]);// F macro hack
            for (byte y = 4; y; y--)// Display four equally sized arrays
            {
                byte * currentVar;
                switch (y)
                {
                case 1:
                    currentVar = configPage6.iacBins;
                    break;
                case 2:
                    currentVar = configPage6.iacOLPWMVal;
                    break;
                case 3:
                    currentVar = configPage6.iacOLStepVal;
                    break;
                case 4:
                    currentVar = configPage6.iacCLValues;
                    break;
                default:
                    break;
                }
                for (byte x = 10; x; x--)
                {
                    Serial.print(currentVar[10 - x]);
                    Serial.print(' ');
                }
                Serial.println();
            }
            for (byte y = 3; y; y--)// Three equally sized arrays
            {
                byte * currentVar;
                switch (y)
                {
                case 1:
                    currentVar = configPage6.iacCrankBins;
                    break;
                case 2:
                    currentVar = configPage6.iacCrankDuty;
                    break;
                case 3:
                    currentVar = configPage6.iacCrankSteps;
                    break;
                default:
                    break;
                }
                for (byte x = 4; x; x--)
                {
                    Serial.print(currentVar[4 - x]);
                    Serial.print(' ');
                }
                Serial.println();
            }
            // Following loop is for remaining byte value of page
            for (pnt_configPage = (byte *)&configPage6.iacCrankBins[3] + 1; pnt_configPage < (byte *)&configPage6 + npage_size[afrSetPage]; pnt_configPage = (byte *)pnt_configPage + 1) {
                Serial.println(*((byte *)pnt_configPage));
            }
            sendComplete = true;
        }
        else {
            pnt_configPage = &configPage6;    //Create a pointer to Page 4 in memory
        }
        break;

    case boostvvtPage:
        if(useChar)
        {
            currentTable = boostTable;
            currentTitleIndex = 121;
        }
        else
        {
            //Need to perform a translation of the values[MAP/TPS][RPM] into the MS expected format
            byte response[80]; //Bit hacky, but send 1 map at a time (Each map is 8x8, so 64 + 8 + 8)

            //Boost table
            for (int x = 0; x < 64; x++) {
                response[x] = boostTable.values[7 - (x / 8)][x % 8];
            }
            for (int x = 64; x < 72; x++) {
                response[x] = byte(boostTable.axisX[(x - 64)] / TABLE_RPM_MULTIPLIER);
            }
            for (int y = 72; y < 80; y++) {
                response[y] = byte(boostTable.axisY[7 - (y - 72)]);
            }
            Serial.write((byte *)&response, 80);
            //VVT table
            for (int x = 0; x < 64; x++) {
                response[x] = vvtTable.values[7 - (x / 8)][x % 8];
            }
            for (int x = 64; x < 72; x++) {
                response[x] = byte(vvtTable.axisX[(x - 64)] / TABLE_RPM_MULTIPLIER);
            }
            for (int y = 72; y < 80; y++) {
                response[y] = byte(vvtTable.axisY[7 - (y - 72)]);
            }
            Serial.write((byte *)&response, 80);
            //Staging table
            for (int x = 0; x < 64; x++) {
                response[x] = stagingTable.values[7 - (x / 8)][x % 8];
            }
            for (int x = 64; x < 72; x++) {
                response[x] = byte(stagingTable.axisX[(x - 64)] / TABLE_RPM_MULTIPLIER);
            }
            for (int y = 72; y < 80; y++) {
                response[y] = byte(stagingTable.axisY[7 - (y - 72)] / TABLE_LOAD_MULTIPLIER);
            }
            Serial.write((byte *)&response, 80);
            sendComplete = true;
        }
        break;

    case seqFuelPage:
        if(useChar)
        {
            currentTable = trim1Table;
            for (int y = 0; y < currentTable.ySize; y++)
            {
                byte axisY = byte(currentTable.axisY[y]);
                if (axisY < 100)
                {
                    Serial.write(" ");
                    if (axisY < 10)
                    {
                        Serial.write(" ");
                    }
                }
                Serial.print(axisY);// Vertical Bins
                Serial.write(" ");
                for (int x = 0; x < currentTable.xSize; x++)
                {
                    byte value = currentTable.values[y][x];
                    if (value < 100)
                    {
                        Serial.write(" ");
                        if (value < 10)
                        {
                            Serial.write(" ");
                        }
                    }
                    Serial.print(value);
                    Serial.write(" ");
                }
                Serial.println("");
            }
            sendComplete = true;
            //Do.... Something?
        }
        else
        {
            //Need to perform a translation of the values[MAP/TPS][RPM] into the MS expected format
            byte response[192]; //Bit hacky, but the size is: (6x6 + 6 + 6) * 4 = 192

            //trim1 table
            for (int x = 0; x < 36; x++) {
                response[x] = trim1Table.values[5 - (x / 6)][x % 6];
            }
            for (int x = 36; x < 42; x++) {
                response[x] = byte(trim1Table.axisX[(x - 36)] / TABLE_RPM_MULTIPLIER);
            }
            for (int y = 42; y < 48; y++) {
                response[y] = byte(trim1Table.axisY[5 - (y - 42)] / TABLE_LOAD_MULTIPLIER);
            }
            //trim2 table
            for (int x = 0; x < 36; x++) {
                response[x + 48] = trim2Table.values[5 - (x / 6)][x % 6];
            }
            for (int x = 36; x < 42; x++) {
                response[x + 48] = byte(trim2Table.axisX[(x - 36)] / TABLE_RPM_MULTIPLIER);
            }
            for (int y = 42; y < 48; y++) {
                response[y + 48] = byte(trim2Table.axisY[5 - (y - 42)] / TABLE_LOAD_MULTIPLIER);
            }
            //trim3 table
            for (int x = 0; x < 36; x++) {
                response[x + 96] = trim3Table.values[5 - (x / 6)][x % 6];
            }
            for (int x = 36; x < 42; x++) {
                response[x + 96] = byte(trim3Table.axisX[(x - 36)] / TABLE_RPM_MULTIPLIER);
            }
            for (int y = 42; y < 48; y++) {
                response[y + 96] = byte(trim3Table.axisY[5 - (y - 42)] / TABLE_LOAD_MULTIPLIER);
            }
            //trim4 table
            for (int x = 0; x < 36; x++) {
                response[x + 144] = trim4Table.values[5 - (x / 6)][x % 6];
            }
            for (int x = 36; x < 42; x++) {
                response[x + 144] = byte(trim4Table.axisX[(x - 36)] / TABLE_RPM_MULTIPLIER);
            }
            for (int y = 42; y < 48; y++) {
                response[y + 144] = byte(trim4Table.axisY[5 - (y - 42)] / TABLE_LOAD_MULTIPLIER);
            }
            Serial.write((byte *)&response, sizeof(response));
            sendComplete = true;
        }
        break;

    case canbusPage:
        //currentTitleIndex = 141;
        if (useChar)
        {
            //To Display Values from Config Page 10
            Serial.println((const __FlashStringHelper *)&pageTitles[103]);//special typecasting to enable suroutine that the F macro uses
            for (pnt_configPage = &configPage9; pnt_configPage < ((byte *)pnt_configPage + 128); pnt_configPage = (byte *)pnt_configPage + 1)
            {
                Serial.println(*((byte *)pnt_configPage));// Displaying byte values of config page 3 up to but not including the first array
            }
            sendComplete = true;
        }
        else {
            pnt_configPage = &configPage9;    //Create a pointer to Page 10 in memory
        }
        break;

    case warmupPage:
        if (useChar)
        {
            sendComplete = true;
        }
        else {
            pnt_configPage = &configPage10;    //Create a pointer to Page 11 in memory
        }
        break;

    default:
        //Just set default Values to avoid warnings
        pnt_configPage = &configPage10;
        currentTable = fuelTable;
        sendComplete = true;
        break;
    }
    if(!sendComplete)
    {
        if (isMap)
        {
            if (useChar)
            {
                do //This is a do while loop that kicks in for the boostvvtPage
                {
                    const char spaceChar = ' ';
                    /*while(pageTitles[currentTitleIndex])
                    {
                     Serial.print(pageTitles[currentTitleIndex]);
                     currentTitleIndex++;
                    }*/
                    Serial.println((const __FlashStringHelper *)&pageTitles[currentTitleIndex]);// F macro hack
                    Serial.println();
                    for (int y = 0; y < currentTable.ySize; y++)
                    {
                        byte axisY = byte(currentTable.axisY[y]);
                        if (axisY < 100)
                        {
                            Serial.write(spaceChar);
                            if (axisY < 10)
                            {
                                Serial.write(spaceChar);
                            }
                        }
                        Serial.print(axisY);// Vertical Bins
                        Serial.write(spaceChar);
                        for (int i = 0; i < currentTable.xSize; i++)
                        {
                            byte value = currentTable.values[y][i];
                            if (value < 100)
                            {
                                Serial.write(spaceChar);
                                if (value < 10)
                                {
                                    Serial.write(spaceChar);
                                }
                            }
                            Serial.print(value);
                            Serial.write(spaceChar);
                        }
                        Serial.println();
                    }
                    Serial.print(F("    "));
                    for (int x = 0; x < currentTable.xSize; x++)// Horizontal bins
                    {
                        byte axisX = byte(currentTable.axisX[x] / 100);
                        if (axisX < 100)
                        {
                            Serial.write(spaceChar);
                            if (axisX < 10)
                            {
                                Serial.write(spaceChar);
                            }
                        }
                        Serial.print(axisX);
                        Serial.write(spaceChar);
                    }
                    Serial.println();
                    if(currentTitleIndex == 121) //Check to see if on boostTable
                    {
                        currentTitleIndex = 132; //Change over to vvtTable mid display
                        currentTable = vvtTable;
                    }
                    else currentTitleIndex = 0;
                } while(currentTitleIndex == 132); //Should never loop unless going to display vvtTable
            } //use char
            else
            {
                //Need to perform a translation of the values[yaxis][xaxis] into the MS expected format
                //MS format has origin (0,0) in the bottom left corner, we use the top left for efficiency reasons
                byte response[MAP_PAGE_SIZE];

                for (int x = 0; x < 256; x++) {
                    response[x] = currentTable.values[15 - (x / 16)][x % 16];    //This is slightly non-intuitive, but essentially just flips the table vertically (IE top line becomes the bottom line etc). Columns are unchanged. Every 16 loops, manually call loop() to avoid potential misses
                }
                //loop();
                for (int x = 256; x < 272; x++) {
                    response[x] = byte(currentTable.axisX[(x - 256)] / TABLE_RPM_MULTIPLIER);    //RPM Bins for VE table (Need to be dvidied by 100)
                }
                //loop();
                for (int y = 272; y < 288; y++) {
                    response[y] = byte(currentTable.axisY[15 - (y - 272)] / TABLE_LOAD_MULTIPLIER);    //MAP or TPS bins for VE table
                }
                //loop();
                Serial.write((byte *)&response, sizeof(response));
            }
        } //is map
        else
        {
            /*if(useChar)
            {
             while(pageTitles[currentTitleIndex])
             {
              Serial.print(pageTitles[currentTitleIndex]);
              currentTitleIndex++;
             }
             Serial.println();
             for(byte x=0;x<page_size;x++) Serial.println(*((byte *)pnt_configPage + x));
            }
            else
            {*/
            //All other bytes can simply be copied from the config table
            //byte response[npage_size[currentPage]];
            for (byte x = 0; x < npage_size[currentPage]; x++)
            {
                //response[x] = *((byte *)pnt_configPage + x);
                Serial.write(*((byte *)pnt_configPage + x)); //Each byte is simply the location in memory of the configPage + the offset + the variable number (x)
            }

            //Serial.write((byte *)&response, npage_size[currentPage]);
            // }
        } //isMap
    } //sendComplete
}

byte getPageValue(byte page, uint16_t valueAddress)
{
    void* pnt_configPage = &configPage2; //Default value is for safety only. Will be changed below if needed.
    uint16_t tempAddress;
    byte returnValue = 0;

    switch (page)
    {
    case veMapPage:
        if( valueAddress < 256) {
            returnValue = fuelTable.values[15 - (valueAddress / 16)][valueAddress % 16];    //This is slightly non-intuitive, but essentially just flips the table vertically (IE top line becomes the bottom line etc). Columns are unchanged. Every 16 loops, manually call loop() to avoid potential misses
        }
        else if(valueAddress < 272) {
            returnValue =  byte(fuelTable.axisX[(valueAddress - 256)] / TABLE_RPM_MULTIPLIER);    //RPM Bins for VE table (Need to be dvidied by 100)
        }
        else if (valueAddress < 288) {
            returnValue = byte(fuelTable.axisY[15 - (valueAddress - 272)] / TABLE_LOAD_MULTIPLIER);    //MAP or TPS bins for VE table
        }
        break;

    case veSetPage:
        pnt_configPage = &configPage2; //Create a pointer to Page 1 in memory
        returnValue = *((byte *)pnt_configPage + valueAddress);
        break;

    case ignMapPage:
        if( valueAddress < 256) {
            returnValue = ignitionTable.values[15 - (valueAddress / 16)][valueAddress % 16];    //This is slightly non-intuitive, but essentially just flips the table vertically (IE top line becomes the bottom line etc). Columns are unchanged. Every 16 loops, manually call loop() to avoid potential misses
        }
        else if(valueAddress < 272) {
            returnValue =  byte(ignitionTable.axisX[(valueAddress - 256)] / TABLE_RPM_MULTIPLIER);    //RPM Bins for VE table (Need to be dvidied by 100)
        }
        else if (valueAddress < 288) {
            returnValue = byte(ignitionTable.axisY[15 - (valueAddress - 272)] / TABLE_LOAD_MULTIPLIER);    //MAP or TPS bins for VE table
        }
        break;

    case ignSetPage:
        pnt_configPage = &configPage4; //Create a pointer to Page 2 in memory
        returnValue = *((byte *)pnt_configPage + valueAddress);
        break;

    case afrMapPage:
        if( valueAddress < 256) {
            returnValue = afrTable.values[15 - (valueAddress / 16)][valueAddress % 16];    //This is slightly non-intuitive, but essentially just flips the table vertically (IE top line becomes the bottom line etc). Columns are unchanged. Every 16 loops, manually call loop() to avoid potential misses
        }
        else if(valueAddress < 272) {
            returnValue =  byte(afrTable.axisX[(valueAddress - 256)] / TABLE_RPM_MULTIPLIER);    //RPM Bins for VE table (Need to be dvidied by 100)
        }
        else if (valueAddress < 288) {
            returnValue = byte(afrTable.axisY[15 - (valueAddress - 272)] / TABLE_LOAD_MULTIPLIER);    //MAP or TPS bins for VE table
        }
        break;

    case afrSetPage:
        pnt_configPage = &configPage6; //Create a pointer to Page 3 in memory
        returnValue = *((byte *)pnt_configPage + valueAddress);
        break;

    case boostvvtPage:

    {
        //Need to perform a translation of the values[MAP/TPS][RPM] into the MS expected format
        if(valueAddress < 80)
        {
            //Boost table
            if(valueAddress < 64) {
                returnValue = boostTable.values[7 - (valueAddress / 8)][valueAddress % 8];
            }
            else if(valueAddress < 72) {
                returnValue = byte(boostTable.axisX[(valueAddress - 64)] / TABLE_RPM_MULTIPLIER);
            }
            else if(valueAddress < 80) {
                returnValue = byte(boostTable.axisY[7 - (valueAddress - 72)]);
            }
        }
        else if(valueAddress < 160)
        {
            tempAddress = valueAddress - 80;
            //VVT table
            if(tempAddress < 64) {
                returnValue = vvtTable.values[7 - (tempAddress / 8)][tempAddress % 8];
            }
            else if(tempAddress < 72) {
                returnValue = byte(vvtTable.axisX[(tempAddress - 64)] / TABLE_RPM_MULTIPLIER);
            }
            else if(tempAddress < 80) {
                returnValue = byte(vvtTable.axisY[7 - (tempAddress - 72)]);
            }
        }
        else
        {
            tempAddress = valueAddress - 160;
            //Staging table
            if(tempAddress < 64) {
                returnValue = stagingTable.values[7 - (tempAddress / 8)][tempAddress % 8];
            }
            else if(tempAddress < 72) {
                returnValue = byte(stagingTable.axisX[(tempAddress - 64)] / TABLE_RPM_MULTIPLIER);
            }
            else if(tempAddress < 80) {
                returnValue = byte(stagingTable.axisY[7 - (tempAddress - 72)] / TABLE_LOAD_MULTIPLIER);
            }
        }
    }
    break;

    case seqFuelPage:

    {
        //Need to perform a translation of the values[MAP/TPS][RPM] into the TS expected format
        if(valueAddress < 48)
        {
            //trim1 table
            if(valueAddress < 36) {
                returnValue = trim1Table.values[5 - (valueAddress / 6)][valueAddress % 6];
            }
            else if(valueAddress < 42) {
                returnValue = byte(trim1Table.axisX[(valueAddress - 36)] / TABLE_RPM_MULTIPLIER);
            }
            else if(valueAddress < 48) {
                returnValue = byte(trim1Table.axisY[5 - (valueAddress - 42)] / TABLE_LOAD_MULTIPLIER);
            }
        }
        else if(valueAddress < 96)
        {
            tempAddress = valueAddress - 48;
            //trim2 table
            if(tempAddress < 36) {
                returnValue = trim2Table.values[5 - (tempAddress / 6)][tempAddress % 6];
            }
            else if(tempAddress < 42) {
                returnValue = byte(trim2Table.axisX[(tempAddress - 36)] / TABLE_RPM_MULTIPLIER);
            }
            else if(tempAddress < 48) {
                returnValue = byte(trim2Table.axisY[5 - (tempAddress - 42)] / TABLE_LOAD_MULTIPLIER);
            }
        }
        else if(valueAddress < 144)
        {
            tempAddress = valueAddress - 96;
            //trim3 table
            if(tempAddress < 36) {
                returnValue = trim3Table.values[5 - (tempAddress / 6)][tempAddress % 6];
            }
            else if(tempAddress < 42) {
                returnValue = byte(trim3Table.axisX[(tempAddress - 36)] / TABLE_RPM_MULTIPLIER);
            }
            else if(tempAddress < 48) {
                returnValue = byte(trim3Table.axisY[5 - (tempAddress - 42)] / TABLE_LOAD_MULTIPLIER);
            }
        }
        else if(valueAddress < 192)
        {
            tempAddress = valueAddress - 144;
            //trim4 table
            if(tempAddress < 36) {
                returnValue = trim4Table.values[5 - (tempAddress / 6)][tempAddress % 6];
            }
            else if(tempAddress < 42) {
                returnValue = byte(trim4Table.axisX[(tempAddress - 36)] / TABLE_RPM_MULTIPLIER);
            }
            else if(tempAddress < 48) {
                returnValue = byte(trim4Table.axisY[5 - (tempAddress - 42)] / TABLE_LOAD_MULTIPLIER);
            }
        }
    }
    break;

    case canbusPage:
        pnt_configPage = &configPage9; //Create a pointer to Page 10 in memory
        returnValue = *((byte *)pnt_configPage + valueAddress);
        break;

    case warmupPage:
        pnt_configPage = &configPage10; //Create a pointer to Page 11 in memory
        returnValue = *((byte *)pnt_configPage + valueAddress);
        break;

    default:
        //Just set default Values to avoid warnings
        pnt_configPage = &configPage10;
        break;
    }
    return returnValue;
}


void receiveCalibration(byte tableID)//stripped down function to keep tunerstudio happy
{
    int OFFSET, DIVISION_FACTOR, BYTES_PER_VALUE, EEPROM_START;

    switch (tableID)
    {
    case 0:
        BYTES_PER_VALUE = 2;
        break;
    case 1:
        BYTES_PER_VALUE = 2;
        break;
    case 2:
        BYTES_PER_VALUE = 1;
        break;
    default:
        BYTES_PER_VALUE = 1;
        break;
    }
    unsigned int x=0;
    for (x = 0; x < 1024; x++)
    {
        if (BYTES_PER_VALUE == 1)
        {
            while ( Serial.available() < 1 ) {}
            Serial.read();
        }
        else
        {
            while ( Serial.available() < 2 ) {}
            Serial.read();
            Serial.read();
        }
    }
}

void testComm()
{
    Serial.write(1);
    return;
}

void commandButtons()
{
    switch (cmdCombined)
    {
    case 256: // cmd is stop
        BIT_CLEAR(currentStatus.testOutputs, 1);
        fire_coil1();
        fire_coil2();
        fire_coil3();
        fire_coil4();
        close_injector1();
        close_injector2();
        close_injector3();
        close_injector4();
        break;

    case 257: // cmd is enable
        // currentStatus.testactive = 1;
        BIT_SET(currentStatus.testOutputs, 1);
        break;
    case 513: // cmd group is for injector1 on actions
        if( BIT_CHECK(currentStatus.testOutputs, 1) ) {
            open_injector1();
        }
        break;
    case 514: // cmd group is for injector1 off actions
        if( BIT_CHECK(currentStatus.testOutputs, 1) ) {
            close_injector1();
        }
        break;
    case 515: // cmd group is for injector1 50% dc actions
        //for (byte dcloop = 0; dcloop < 11; dcloop++)
        //{
        //  digitalWrite(PIN_INJ1, HIGH);
        //  delay(500);
        //  digitalWrite(PIN_INJ1, LOW);
        //  delay(500);
        //}
        break;
    case 516: // cmd group is for injector2 on actions
        if( BIT_CHECK(currentStatus.testOutputs, 1) ) {
            open_injector2();
        }
        break;
    case 517: // cmd group is for injector2 off actions
        if( BIT_CHECK(currentStatus.testOutputs, 1) ) {
            close_injector2();
        }
        break;
    case 518: // cmd group is for injector2 50%dc actions

        break;
    case 519: // cmd group is for injector3 on actions
        if( BIT_CHECK(currentStatus.testOutputs, 1) ) {
            open_injector3();
        }
        break;
    case 520: // cmd group is for injector3 off actions
        if( BIT_CHECK(currentStatus.testOutputs, 1) ) {
            close_injector3();
        }
        break;
    case 521: // cmd group is for injector3 50%dc actions

        break;
    case 522: // cmd group is for injector4 on actions
        if( BIT_CHECK(currentStatus.testOutputs, 1) ) {
            open_injector4();
        }
        break;
    case 523: // cmd group is for injector4 off actions
        if( BIT_CHECK(currentStatus.testOutputs, 1) ) {
            close_injector4();
        }
        break;
    case 524: // cmd group is for injector4 50% dc actions

        break;
    case 769: // cmd group is for spark1 on actions
        break;
    case 770: // cmd group is for spark1 off actions
        break;
    case 771: // cmd group is for spark1 50%dc actions

        break;
    case 772: // cmd group is for spark2 on actions
        if( BIT_CHECK(currentStatus.testOutputs, 1) ) {
            //digitalWrite(PIN_IGN2, CONFIG_COIL_ON);
        }
        break;
    case 773: // cmd group is for spark2 off actions
        if( BIT_CHECK(currentStatus.testOutputs, 1) ) {
            //digitalWrite(PIN_IGN2, CONFIG_COIL_OFF);
        }
        break;
    case 774: // cmd group is for spark2 50%dc actions

        break;
    case 775: // cmd group is for spark3 on actions
        if( BIT_CHECK(currentStatus.testOutputs, 1) ) {
            //digitalWrite(PIN_IGN3, CONFIG_COIL_ON);
        }
        break;
    case 776: // cmd group is for spark3 off actions
        if( BIT_CHECK(currentStatus.testOutputs, 1) ) {
            //digitalWrite(PIN_IGN3, CONFIG_COIL_OFF);
        }
        break;
    case 777: // cmd group is for spark3 50%dc actions

        break;
    case 778: // cmd group is for spark4 on actions
        if( BIT_CHECK(currentStatus.testOutputs, 1) ) {
            //digitalWrite(PIN_IGN4, CONFIG_COIL_ON);
        }
        break;
    case 779: // cmd group is for spark4 off actions
        if( BIT_CHECK(currentStatus.testOutputs, 1) ) {
            //digitalWrite(PIN_IGN4, CONFIG_COIL_OFF);
        }
        break;
    case 780: // cmd group is for spark4 50%dc actions

    default:
        break;
    }
}
