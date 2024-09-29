/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#include <inttypes.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <PS1080.h>
#include <OpenNI.h>
#include <XnOS.h>
#include <XnStreamParams.h>

using namespace std;
#if(XN_PLATFORM == XN_PLATFORM_WIN32)
#pragma warning(disable: 4996)
#pragma warning(disable: 4819)
#endif


typedef bool (*cbfunc)(openni::Device& Device, vector<string>& Command);

map<string, cbfunc> cbs;
map<string, cbfunc> mnemonics;
map<string, string>  helps;

XnVersions g_DeviceVersion;
const char* g_openMode;
openni::VideoStream g_depthStream;
openni::VideoStream g_colorStream;

const XnUInt16 REQUIRE_MODE_PS = 0x0001;
const XnUInt16 REQUIRE_MODE_MAINTENANCE = 0x0002;
const XnUInt16 REQUIRE_MODE_ANY = 0xFFFF;
std::vector<string> g_vecAEOptionsName;
#define SWITCH_MODE_WAIT	3000

bool isStringEqual(string str1, string str2)
{
    return str1.compare(str2) == 0;
}

bool atoi2(const char* str, int* pOut)
{
    int output = 0;
    int base = 10;
    int start = 0;

    if (strlen(str) > 1 && str[0] == '0' && str[1] == 'x')
    {
        start = 2;
        base = 16;
    }

    for (size_t i = start; i < strlen(str); i++)
    {
        output *= base;
        if (str[i] >= '0' && str[i] <= '9')
            output += str[i] - '0';
        else if (base == 16 && str[i] >= 'a' && str[i] <= 'f')
            output += 10 + str[i] - 'a';
        else if (base == 16 && str[i] >= 'A' && str[i] <= 'F')
            output += 10 + str[i] - 'A';
        else
            return false;
    }
    *pOut = output;
    return true;
}


void mainloop(openni::Device& Device, istream& istr, bool prompt)
{
    char buf[256];
    string str;

    vector<string> Command;

    while (istr.good())
    {
        if (prompt)
            cout << "> ";
        Command.clear();
        istr.getline(buf, 256);
        str = buf;
        size_t previous = 0, next = 0;

        while (1)
        {
            next = str.find(' ', previous);

            if (next != previous && previous != str.size())
                Command.push_back(str.substr(previous, next - previous));

            if (next == str.npos)
                break;

            previous = next + 1;
        }

        if (Command.size() > 0)
        {
            if (Command[0][0] == ';')
                continue;

            for (unsigned int i = 0; i < Command[0].size(); i++)
                Command[0][i] = (char)tolower(Command[0][i]);

            if (cbs.find(Command[0]) != cbs.end())
            {
                if (!(*cbs[Command[0]])(Device, Command))
                    return;
            }
            else if (mnemonics.find(Command[0]) != mnemonics.end())
            {
                if (!(*mnemonics[Command[0]])(Device, Command))
                    return;
            }
            else
            {
                cout << "Unknown command \"" << Command[0] << "\"" << endl;
            }
        }
    }
}

bool byebye(openni::Device& /*Device*/, vector<string>& /*Command*/)
{
    cout << "Bye bye" << endl;
    return false;
}

bool PrintVersion(openni::Device& Device, vector<string>& /*Command*/)
{
    openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_VERSION, &g_DeviceVersion);
    if (rc != openni::STATUS_OK)
    {
        printf("Error: %s\n", openni::OpenNI::getExtendedError());
        return true;
    }

    XnChar strPlatformString[XN_DEVICE_MAX_STRING_LENGTH] = { 0 };
    int size = sizeof(strPlatformString);
    rc = Device.getProperty(XN_MODULE_PROPERTY_SENSOR_PLATFORM_STRING, strPlatformString, &size);
    if (rc != openni::STATUS_OK)
    {
        printf("Error: %s\n", openni::OpenNI::getExtendedError());
        return true;
    }

    printf("Firmware: V%d.%d.%d", g_DeviceVersion.nMajor, g_DeviceVersion.nMinor, g_DeviceVersion.nBuild);
    if (strPlatformString[0] != '\0')
    {
        printf("-%s", strPlatformString);
    }

    printf("; SDK:%s;", ONI_BRIEF_VERSION_STRING);
    printf(" Chip: 0x%08x\nFPGA: 0x%x; System: 0x%x; ",
        g_DeviceVersion.nChip, g_DeviceVersion.nFPGA, g_DeviceVersion.nSystemVersion);

    OBPublicBoardVersion publicBoardVersion = { 0 };
    rc = Device.getProperty(XN_MODULE_PROPERTY_PUBLIC_BOARD_VERSION, &publicBoardVersion);
    if (rc == openni::STATUS_OK)
    {
        printf("Public Board Version: ");
        for (uint32_t i = 0; i < sizeof(publicBoardVersion); i++)
        {
            printf("%c", publicBoardVersion.cVersion[i]);
        }
        printf("; ");
    }

    printf("FWBaseLine: ");
    if (g_DeviceVersion.FWVer == XN_SENSOR_FW_VER_0_17)
    {
        printf("V0.17");
    }
    else if (g_DeviceVersion.FWVer == XN_SENSOR_FW_VER_1_1)
    {
        printf("V1.1");
    }
    else if (g_DeviceVersion.FWVer == XN_SENSOR_FW_VER_1_2)
    {
        printf("V1.2");
    }
    else if (g_DeviceVersion.FWVer == XN_SENSOR_FW_VER_3_0)
    {
        printf("V3.0");
    }
    else if (g_DeviceVersion.FWVer == XN_SENSOR_FW_VER_4_0)
    {
        printf("V4.0");
    }
    else if (g_DeviceVersion.FWVer == XN_SENSOR_FW_VER_5_0)
    {
        printf("V5.0");
    }
    else if (g_DeviceVersion.FWVer == XN_SENSOR_FW_VER_5_1)
    {
        printf("V5.1");
    }
    else if (g_DeviceVersion.FWVer == XN_SENSOR_FW_VER_5_2)
    {
        printf("V5.2");
    }
    else if (g_DeviceVersion.FWVer == XN_SENSOR_FW_VER_5_3)
    {
        printf("V5.3");
    }
    else if (g_DeviceVersion.FWVer == XN_SENSOR_FW_VER_5_4)
    {
        printf("V5.4");
    }
    else if (g_DeviceVersion.FWVer == XN_SENSOR_FW_VER_5_5)
    {
        printf("V5.5");
    }
    else if (g_DeviceVersion.FWVer == XN_SENSOR_FW_VER_5_6)
    {
        printf("V5.6");
    }
    else if (g_DeviceVersion.FWVer == XN_SENSOR_FW_VER_5_7)
    {
        printf("V5.7");
    }
    else if (g_DeviceVersion.FWVer == XN_SENSOR_FW_VER_5_8)
    {
        printf("V5.8");
    }
    else if (g_DeviceVersion.FWVer == XN_SENSOR_FW_VER_5_9)
    {
        printf("V5.9");
    }
    else if (g_DeviceVersion.FWVer == XN_SENSOR_FW_VER_UNKNOWN)
    {
        printf("Unknown");
    }

    printf("; Board: ");
    if (g_DeviceVersion.HWVer == XN_SENSOR_HW_VER_CDB_10)
    {
        printf("CDB1.0");
    }
    else if (g_DeviceVersion.HWVer == XN_SENSOR_HW_VER_FPDB_10)
    {
        printf("FPDB1.0");
    }
    else if (g_DeviceVersion.HWVer == XN_SENSOR_HW_VER_RD_3)
    {
        printf("RD3.0");
    }
    else if (g_DeviceVersion.HWVer == XN_SENSOR_HW_VER_RD_5)
    {
        printf("RD5.0");
    }
    else if (g_DeviceVersion.HWVer == XN_SENSOR_HW_VER_RD1081)
    {
        printf("RD1081");
    }
    else if (g_DeviceVersion.HWVer == XN_SENSOR_HW_VER_RD1082)
    {
        printf("RD1082");
    }
    else if (g_DeviceVersion.HWVer == XN_SENSOR_HW_VER_RD109)
    {
        printf("RD109");
    }
    else if (g_DeviceVersion.HWVer == XN_SENSOR_HW_VER_UNKNOWN)
    {
        printf("Unknown");
    }

    printf("; ChipType: ");
    if (g_DeviceVersion.ChipVer == XN_SENSOR_CHIP_VER_PS1000)
    {
        printf("PS1000");
    }
    else if (g_DeviceVersion.ChipVer == XN_SENSOR_CHIP_VER_PS1080)
    {
        printf("PS1080");
    }
    else if (g_DeviceVersion.ChipVer == XN_SENSOR_CHIP_VER_PS1080A6)
    {
        printf("PS1080A6");
    }

    else if (g_DeviceVersion.ChipVer == XN_SENSOR_CHIP_VER_MX6000)
    {
        printf("MX6000");
    }

    else if (g_DeviceVersion.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
    {
        printf("Dual MX6000");
    }

    else if (g_DeviceVersion.ChipVer == XN_SENSOR_CHIP_VER_UNKNOWN)
    {
        printf("Unknown");
    }

    printf("\n");

    return true;
}

bool DeleteFile(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() != 2)
    {
        printf("Usage: %s <file id>\n", Command[0].c_str());
        return true;
    }

    int nId;
    if (!atoi2(Command[1].c_str(), &nId))
    {
        printf("Id  (%s) isn't a number\n", Command[1].c_str());
        return true;
    }
    printf("Deleting file id %d: ", nId);

    openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_DELETE_FILE, (XnUInt64)nId);
    if (rc != openni::STATUS_OK)
    {
        printf("Failed: %s\n", openni::OpenNI::getExtendedError());
    }

    printf("Done\n");

    return true;
}

bool Attrib(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() < 2)
    {
        printf("Usage: %s <id> [<+-><attributes> ...]\n", Command[0].c_str());
        printf("\t+r - Set read only. -r - Set not read only\n");
        return true;
    }
    int nId;
    if (!atoi2(Command[1].c_str(), &nId))
    {
        printf("Id (%s) isn't a number\n", Command[1].c_str());
        return true;
    }

    XnFlashFile Files[100];

    XnFlashFileList FileList;
    FileList.pFiles = (XnFlashFile*)Files;
    FileList.nFiles = 100;

    openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_FILE_LIST, &FileList);
    if (rc != openni::STATUS_OK)
    {
        printf("Couldn't get file list: %s\n", openni::OpenNI::getExtendedError());
        return true;
    }

    XnFlashFile* pEntry = NULL;

    for (int i = 0; i < FileList.nFiles; i++)
    {
        if (FileList.pFiles[i].nId == nId)
            pEntry = &FileList.pFiles[i];
    }
    if (pEntry == NULL)
    {
        printf("No such id %d\n", nId);
        return true;
    }

    if (Command.size() == 2)
    {
        // Get
        printf("0x%04x: ", pEntry->nAttributes);

        // All attributes
        printf("%cr ", (pEntry->nAttributes & XnFileAttributeReadOnly) ? '+' : '-');
        // Done
        printf("\n");
        return true;
    }

    XnUInt16 nNewAttributes = pEntry->nAttributes;

    for (unsigned int i = 2; i < Command.size(); i++)
    {
        if (Command[i] == "+r")
            nNewAttributes |= XnFileAttributeReadOnly;
        else if (Command[i] == "-r")
            nNewAttributes &= ~XnFileAttributeReadOnly;
        else
        {
            printf("Unknown attribute: %s\n", Command[i].c_str());
            return true;
        }
    }

    if (nNewAttributes == pEntry->nAttributes)
    {
        printf("Target attributes is same as existing one!\n");
        return true;
    }

    XnFileAttributes Attribs;
    Attribs.nId = pEntry->nId;
    Attribs.nAttribs = nNewAttributes;

    rc = Device.setProperty(XN_MODULE_PROPERTY_FILE_ATTRIBUTES, Attribs);
    if (rc != openni::STATUS_OK)
    {
        printf("Couldn't change attributes: %s\n", openni::OpenNI::getExtendedError());
        return true;
    }

    return true;
}

bool Connect(openni::Device& Device)
{
    openni::Status rc = openni::STATUS_OK;

    int nCounter = 10;
    while (nCounter-- != 0)
    {
        rc = Device._openEx(NULL, g_openMode);
        if (rc == openni::STATUS_OK || rc == openni::STATUS_NO_DEVICE)
            break;

        // wait and try again
        xnOSSleep(1000);
    }

    if (rc != openni::STATUS_OK)
    {
        printf("Open failed: %s\n", openni::OpenNI::getExtendedError());
        return false;
    }

    return true;
}

bool Reconnect(openni::Device& Device, vector<string>& /*Command*/)
{
    Device.close();

    if (!Connect(Device))
    {
        return false;
    }

    printf("Reconnected\n");

    return true;
}

bool Sleep(openni::Device& /*Device*/, vector<string>& Command)
{
    if (Command.size() != 2)
    {
        printf("Usage: %s <ms>\n", Command[0].c_str());
        return true;
    }

    XnInt32 nMilliSeconds;
    if (!atoi2(Command[1].c_str(), &nMilliSeconds))
    {
        printf("%s doesn't describe a time quantity in milliseconds\n", Command[1].c_str());
        return true;
    }

    xnOSSleep(nMilliSeconds);
    return true;
}

#define NUM_OF_FILE_TYPES 31
bool FileList(openni::Device& Device, vector<string>& /*Command*/)
{
    const char *FileTypes[NUM_OF_FILE_TYPES] =
    {
        "FILE_TABLE",
        "SCRATCH_FILE",
        "BOOT_SECTOR",
        "BOOT_MANAGER",
        "CODE_DOWNLOADER",
        "MONITOR",
        "APPLICATION",
        "FIXED_PARAMS",
        "DESCRIPTORS",
        "DEFAULT_PARAMS",
        "IMAGE_CMOS",
        "DEPTH_CMOS",
        "ALGORITHM_PARAMS",
        "QVGA_REFERENCE",
        "VGA_REFERENCE",
        "MAINTENANCE",
        "DEBUG_PARAMS",
        "PRIME_PROCESSOR",
        "GAIN_CONTROL",
        "REG_PARAMS",
        "ID_PARAMS",
        "TEC_PARAMS",
        "APC_PARAMS",
        "SAFETY_PARAMS",
        "PRODUCTION_FILE",
        "UPGRADE_IN_PROGRESS",
        "WAVELENGTH_CORRECTION",
        "GMC_REF_OFFSET",
        "NESA_PARAMS",
        "SENSOR_FAULT",
        "VENDOR_DATA",
    };

    XnFlashFile Files[100];


    XnFlashFileList FileList;
    FileList.pFiles = (XnFlashFile*)Files;
    FileList.nFiles = 100;

    openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_FILE_LIST, &FileList);
    if (rc != openni::STATUS_OK)
    {
        printf("Error: %s\n", openni::OpenNI::getExtendedError());
        return true;
    }

    printf("%-3s %-22s %-8s    %-8s   %-8s %-8s\n", "Id", "Type", "Version", "Offset", "SizeInWords", "Crc");
    for (int i = 0; i < FileList.nFiles; i++)
    {
        XnFlashFile *Entry = &FileList.pFiles[i];
        const char *TypeCaption = "Unknown Type";
        if (Entry->nType < NUM_OF_FILE_TYPES)
            TypeCaption = FileTypes[Entry->nType];
        printf("%-3d %-22s %2x.%02x.%04x  0x%-8X %-8d    0x%-8X", Entry->nId, TypeCaption, Entry->nVersion >> 24, (Entry->nVersion >> 16) & 0xff, Entry->nVersion & 0xffff, Entry->nOffset, Entry->nSize, Entry->nCrc);
        if (Entry->nAttributes & 0x8000)
        {
            printf(" [R]");
        }
        printf("\n");
    }

    return true;
}

bool Reset(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() != 2)
    {
        printf("Usage: %s <type>\n", Command[0].c_str());
        printf("          power|soft\n");
        return true;
    }

    XnParamResetType Type;
    if (Command[1] == "power")
        Type = XN_RESET_TYPE_POWER;
    else if (Command[1] == "soft")
        Type = XN_RESET_TYPE_SOFT;
    else
    {
        printf("Unknown reset type (power|soft)\n");
        return true;
    }

    openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_RESET, (XnUInt64)Type);
    if (rc != openni::STATUS_OK)
    {
        printf("Error: %s\n", openni::OpenNI::getExtendedError());
    }

    return true;
}

bool Log(openni::Device& Device, vector<string>& /*Command*/)
{
    XnUChar LogBuffer[XN_MAX_LOG_SIZE] = { 0 };
    XnBool bAll = true;
    openni::Status rc;

    do
    {
        LogBuffer[0] = '\0';
        int size = sizeof(LogBuffer);
        rc = Device.getProperty(XN_MODULE_PROPERTY_FIRMWARE_LOG, LogBuffer, &size);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
            break;
        }

        if (LogBuffer[0] != '\0')
        {
            printf("%s", LogBuffer);
        }
        else
        {
            bAll = false;
        }
    } while (bAll);

    return true;
}

bool Script(openni::Device& Device, vector<string>& Command)
{

    if (Command.size() != 2)
    {
        cout << "Usage: " << Command[0] << " <file>" << endl;
        return true;
    }
    string filename = "";

    for (unsigned int i = 1; i < Command.size(); i++)
    {
        if (i != 1)
            filename += " ";
        filename += Command[i];
    }

    ifstream ifs;
    ifs.open(filename.c_str());
    if (!ifs.good())
    {
        cout << "Bad file" << endl;
        return true;
    }
    mainloop(Device, ifs, false);
    ifs.close();
    return true;
}

bool Help(openni::Device& /*Device*/, vector<string>& /*Command*/)
{
    for (map<string, cbfunc>::iterator iter = cbs.begin(); iter != cbs.end(); ++iter)
    {
        cout << "\"" << iter->first << "\" - " << helps[iter->first] << endl;
    }

    return true;
}

bool SetGeneralParam(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() != 3)
    {
        cout << "Usage: " << Command[0] << " <param> <value>" << endl;
        return true;
    }

    int nParam, nValue;

    if (!atoi2(Command[1].c_str(), &nParam))
    {
        printf("Don't understand %s as a parameter\n", Command[1].c_str());
        return true;
    }
    if (!atoi2(Command[2].c_str(), &nValue))
    {
        printf("Don't understand %s as a value\n", Command[2].c_str());
        return true;
    }

    XnInnerParamData Param;
    Param.nParam = (unsigned short)nParam;
    Param.nValue = (unsigned short)nValue;

    openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_FIRMWARE_PARAM, Param);
    if (rc != openni::STATUS_OK)
    {
        printf("%s\n", openni::OpenNI::getExtendedError());
    }
    else
    {
        printf("Done\n");
    }

    return true;
}

bool GetGeneralParam(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() != 2)
    {
        cout << "Usage: " << Command[0] << " <param>" << endl;
        return true;
    }

    int nParam;
    if (!atoi2(Command[1].c_str(), &nParam))
    {
        printf("Don't understand %s as a parameter\n", Command[1].c_str());
        return true;
    }

    XnInnerParamData Param;
    Param.nParam = (unsigned short)nParam;
    Param.nValue = (unsigned short)0;

    openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_FIRMWARE_PARAM, &Param);
    if (rc != openni::STATUS_OK)
    {
        printf("%s\n", openni::OpenNI::getExtendedError());
    }
    else
    {
        cout << "Param[" << Param.nParam << "] = " << Param.nValue << endl;
    }

    return true;
}

bool WriteI2C(openni::Device& Device, vector<string>& Command, XnControlProcessingData& I2C)
{
    if (Command.size() != 5)
    {
        cout << "Usage: " << Command[0] << " " << Command[1] << " <cmos> <register> <value>" << endl;
        return true;
    }

    int nRegister, nValue;
    if (!atoi2(Command[3].c_str(), &nRegister))
    {
        printf("Don't understand %s as a register\n", Command[3].c_str());
        return true;
    }
    if (!atoi2(Command[4].c_str(), &nValue))
    {
        printf("Don't understand %s as a value\n", Command[4].c_str());
        return true;
    }
    I2C.nRegister = (unsigned short)nRegister;
    I2C.nValue = (unsigned short)nValue;

    int nParam = 0;

    int command;
    if (!atoi2(Command[2].c_str(), &command))
    {
        printf("cmos should be 0 (depth) or 1 (image)\n");
        return true;
    }

    if (command == 1)
        nParam = XN_MODULE_PROPERTY_DEPTH_CONTROL;
    else if (command == 0)
        nParam = XN_MODULE_PROPERTY_IMAGE_CONTROL;
    else
    {
        cout << "cmos must be 0/1" << endl;
        return true;
    }

    openni::Status rc = Device.setProperty(nParam, I2C);
    if (rc != openni::STATUS_OK)
    {
        printf("%s\n", openni::OpenNI::getExtendedError());
    }

    return true;
}

bool ReadI2C(openni::Device& Device, vector<string>& Command, XnControlProcessingData& I2C)
{
    if (Command.size() != 4)
    {
        cout << "Usage: " << Command[0] << " " << Command[1] << " <cmos> <register>" << endl;
        return true;
    }

    int nRegister;
    if (!atoi2(Command[3].c_str(), &nRegister))
    {
        printf("Don't understand %s as a register\n", Command[3].c_str());
        return true;
    }
    I2C.nRegister = (unsigned short)nRegister;

    int nParam = 0;

    int command;
    if (!atoi2(Command[2].c_str(), &command))
    {
        cout << "cmos must be 0/1" << endl;
        return true;
    }

    if (command == 1)
        nParam = XN_MODULE_PROPERTY_DEPTH_CONTROL;
    else if (command == 0)
        nParam = XN_MODULE_PROPERTY_IMAGE_CONTROL;
    else
    {
        cout << "cmos must be 0/1" << endl;
        return true;
    }

    if (Device.getProperty(nParam, &I2C) != openni::STATUS_OK)
    {
        cout << "GetParam failed!" << endl;
        return true;
    }

    cout << "I2C(" << command << ")[0x" << hex << I2C.nRegister << "] = 0x" << hex << I2C.nValue << endl;

    return true;
}

bool GeneralI2C(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() < 2)
    {
        cout << "Usage: " << Command[0] << " <read/write> ..." << endl;
        return true;
    }
    XnControlProcessingData I2C;

    if (Command[1] == "read")
    {
        return ReadI2C(Device, Command, I2C);
    }
    else if (Command[1] == "write")
    {
        return WriteI2C(Device, Command, I2C);
    }

    cout << "Usage: " << Command[0] << " <read/write> ..." << endl;
    return true;
}

bool WriteAHB(openni::Device& Device, vector<string>& Command, XnAHBData& AHB)
{
    if (Command.size() != 5)
    {
        cout << "Usage: " << Command[0] << " " << Command[1] << " <register> <value> <mask>" << endl;
        return true;
    }

    int nRegister, nValue, nMask;

    if (!atoi2(Command[2].c_str(), &nRegister))
    {
        printf("Can't understand %s as register\n", Command[2].c_str());
        return true;
    }
    if (!atoi2(Command[3].c_str(), &nValue))
    {
        printf("Can't understand %s as value\n", Command[3].c_str());
        return true;
    }
    if (!atoi2(Command[4].c_str(), &nMask))
    {
        printf("Can't understand %s as mask\n", Command[4].c_str());
        return true;
    }


    AHB.nRegister = nRegister;
    AHB.nValue = nValue;
    AHB.nMask = nMask;

    openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_AHB, AHB);
    if (rc != openni::STATUS_OK)
    {
        printf("%s\n", openni::OpenNI::getExtendedError());
    }

    return true;
}

bool ReadAHB(openni::Device& Device, vector<string>& Command, XnAHBData& AHB)
{
    if (Command.size() != 3)
    {
        cout << "Usage: " << Command[0] << " " << Command[1] << " <register>" << endl;
        return true;
    }

    int nRegister;
    if (!atoi2(Command[2].c_str(), &nRegister))
    {
        printf("Can't understand %s as register\n", Command[2].c_str());
        return true;
    }

    AHB.nRegister = nRegister;
    AHB.nValue = 0;

    openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_AHB, &AHB);
    if (rc != openni::STATUS_OK)
    {
        printf("%s\n", openni::OpenNI::getExtendedError());
    }
    else
    {
        cout << "AHB[0x" << hex << AHB.nRegister << "] = 0x" << hex << AHB.nValue << endl;
    }

    return true;
}

bool GeneralAHB(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() < 2)
    {
        cout << "Usage: " << Command[0] << " <read/write> ..." << endl;
        return true;
    }
    XnAHBData AHB;

    if (Command[1] == "read")
    {
        return ReadAHB(Device, Command, AHB);
    }
    else if (Command[1] == "write")
    {
        return WriteAHB(Device, Command, AHB);
    }

    cout << "Usage: " << Command[0] << " <read/write> ..." << endl;
    return true;
}

bool Upload(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() < 3)
    {
        cout << "Usage: " << Command[0] << " <offset> <file> [ro]" << endl;
        return true;
    }
    int nOffset;

    if (!atoi2(Command[1].c_str(), &nOffset))
    {
        printf("Can't understand %s as offset\n", Command[1].c_str());
        return true;
    }

    XnUInt16 nAttributes = 0;

    if (Command.size() >= 4)
    {
        if (Command[3] == "ro")
        {
            nAttributes |= XnFileAttributeReadOnly;
        }
    }

    XnParamFileData ParamFile;
    ParamFile.nOffset = nOffset;
    ParamFile.strFileName = Command[2].c_str();
    ParamFile.nAttributes = nAttributes;

    openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_FILE, ParamFile);
    if (rc != openni::STATUS_OK)
    {
        printf("%s\n", openni::OpenNI::getExtendedError());
    }

    return true;
}

bool Download(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() < 3)
    {
        cout << "Usage: " << Command[0] << " <id> <file>" << endl;
        return true;
    }
    int nId;

    if (!atoi2(Command[1].c_str(), &nId))
    {
        printf("Can't understand %s as id\n", Command[1].c_str());
        return true;
    }
    XnParamFileData ParamFile = { (uint32_t)nId, Command[2].c_str() };

    openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_FILE, &ParamFile);
    if (rc != openni::STATUS_OK)
    {
        printf("%s\n", openni::OpenNI::getExtendedError());
    }

    return true;
}

bool Filter(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() < 2)
    {
        cout << "Usage: " << Command[0] << " <get/set> ..." << endl;
        return true;
    }

    XnInt32 nFilter;
    openni::Status rc;

    if (Command[1] == "get")
    {
        XnUInt64 nValue;
        rc = Device.getProperty(XN_MODULE_PROPERTY_FIRMWARE_LOG_FILTER, &nValue);
        if (rc == openni::STATUS_OK)
        {
            nFilter = (XnInt32)nValue;
            printf("Filter 0x%04x\n", nFilter);
        }
        else
        {
            printf("Failed: %s\n", openni::OpenNI::getExtendedError());
        }
    }
    else if (Command[1] == "set")
    {
        if (Command.size() < 3)
        {
            cout << "Usage: " << Command[0] << " set <filter>" << endl;
            return true;
        }
        if (!atoi2(Command[2].c_str(), &nFilter))
        {
            printf("Can't understand %s as filter\n", Command[2].c_str());
            return true;
        }
        rc = Device.setProperty(XN_MODULE_PROPERTY_FIRMWARE_LOG_FILTER, (XnUInt64)nFilter);
        if (rc == openni::STATUS_OK)
        {
            printf("Done.\n");
        }
        else
        {
            printf("Failed: %s\n", openni::OpenNI::getExtendedError());
        }
    }
    else
    {
        cout << "Usage: " << Command[0] << " <get/set> ..." << endl;
    }

    return true;

}

bool Led(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() < 3)
    {
        cout << "Usage: " << Command[0] << " <id> <on|off>" << endl;
        return true;
    }

    int nLedId;
    if (!atoi2(Command[1].c_str(), &nLedId))
    {
        printf("Can't understand '%s' as LED id\n", Command[1].c_str());
        return true;
    }

    int nState;
    if (Command[2] == "on")
        nState = 1;
    else if (Command[2] == "off")
        nState = 0;
    else
    {
        printf("State must be 'on' or 'off'!\n");
        return true;
    }

    XnLedState ledState;
    ledState.nLedID = (uint16_t)nLedId;
    ledState.nState = (uint16_t)nState;

    openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_LED_STATE, ledState);
    if (rc == openni::STATUS_OK)
    {
        printf("Done.\n");
    }
    else
    {
        printf("Failed: %s\n", openni::OpenNI::getExtendedError());
    }
    return true;
}

bool ReadFixed(openni::Device& Device, vector<string>& Command)
{
    int nParam = -1;
    if (Command.size() > 1)
    {
        if (!atoi2(Command[1].c_str(), &nParam))
        {
            printf("Don't understand %s as a parameter\n", Command[1].c_str());
            return true;
        }
    }

    XnUInt32 anFixedParams[100];
    XnDynamicSizeBuffer buffer;
    buffer.nMaxSize = sizeof(anFixedParams);
    buffer.pData = anFixedParams;
    buffer.nDataSize = 0;

    openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_FIXED_PARAMS, &buffer);
    if (rc != openni::STATUS_OK)
    {
        printf("%s\n", openni::OpenNI::getExtendedError());
    }
    else
    {
        if (nParam != -1)
        {
            if (buffer.nDataSize < nParam * sizeof(XnUInt32))
            {
                cout << "Invalid index! Last index is " << buffer.nDataSize / sizeof(XnUInt32) - 1 << endl;
            }
            else
            {
                cout << "Fixed Param [" << nParam << "] = " << anFixedParams[nParam] << endl;
            }
        }
        else
        {
            for (XnUInt32 i = 0; i < buffer.nDataSize / sizeof(XnUInt32); ++i)
            {
                cout << "Fixed Param [" << i << "] = " << anFixedParams[i] << endl;
            }
        }
    }

    return true;
}

bool Debug(openni::Device& /*Device*/, vector<string>& Command)
{
    for (unsigned int i = 0; i < Command.size(); i++)
        cout << i << ". \"" << Command[i] << "\"" << endl;

    return true;
}

void RegisterCB(string cmd, cbfunc func, const string& strHelp)
{
    for (unsigned int i = 0; i < cmd.size(); i++)
        cmd[i] = (char)tolower(cmd[i]);
    cbs[cmd] = func;
    helps[cmd] = strHelp;
}

void RegisterMnemonic(string strMnemonic, string strCommand)
{
    for (unsigned int i = 0; i < strCommand.size(); i++)
        strCommand[i] = (char)tolower(strCommand[i]);
    for (unsigned int i = 0; i < strMnemonic.size(); i++)
        strMnemonic[i] = (char)tolower(strMnemonic[i]);

    if (cbs.find(strCommand) != cbs.end())
    {
        mnemonics[strMnemonic] = cbs[strCommand];
    }
}

bool ReadFlash(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() < 4)
    {
        cout << "Usage: " << Command[0] << " <offset> <size> <file>" << endl;
        return true;
    }

    XnParamFlashData ParamFlash;

    if (!atoi2(Command[1].c_str(), (int*)&ParamFlash.nOffset))
    {
        printf("Can't understand %s as offset\n", Command[1].c_str());
        return true;
    }

    if (!atoi2(Command[2].c_str(), (int*)&ParamFlash.nSize))
    {
        printf("Can't understand %s as size\n", Command[1].c_str());
        return true;
    }

    int nSizeInBytes = ParamFlash.nSize * sizeof(XnUInt16);
    ParamFlash.pData = new XnUChar[nSizeInBytes];

    openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_FLASH_CHUNK, &ParamFlash);
    if (rc != openni::STATUS_OK)
    {
        printf("%s\n", openni::OpenNI::getExtendedError());
    }

    xnOSSaveFile(Command[3].c_str(), ParamFlash.pData, nSizeInBytes);

    return true;
}

bool RunBIST(openni::Device& Device, vector<string>& Command)
{
    string sBistTests[] =
    {
        "ImageCmos",
        "IrCmos",
        "Potentiometer",
        "Flash",
        "FlashFull",
        "Projector",
        "Tec",
        "NESA",
        "NESAUnlimited"
    };

    int nBistTests = sizeof(sBistTests) / sizeof(string);

    string sBistErrors[] =
    {
        "Ram",
        "Ir Cmos Control Bus",
        "Ir Cmos Data Bus",
        "Ir Cmos Bad Version",
        "Ir Cmos Reset",
        "Ir Cmos Trigger",
        "Ir Cmos Strobe",
        "Color Cmos Control Bus",
        "Color Cmos Data Bus",
        "Color Cmos Bad Version",
        "Color Cmos Reset",
        "Flash Write Line",
        "Flash Test",
        "Potentiometer Control Bus",
        "Potentiometer",
        "Audio Test",
        "Projector LD Failure",
        "Projector LD Failsafe Trigger",
        "Projector Failsafe High Path",
        "Projector Failsafe Low Path",
        "Heater Crossed",
        "Heater Disconnected",
        "Cooler Crossed",
        "Cooler Disconnected",
        "TEC still initializing",
        "TEC out of range",
        "NESA",
    };

    int nBistErrors = sizeof(sBistErrors) / sizeof(string);

    XnBist bist;
    bist.nTestsMask = 0;

    bool bShowUsage = false;

    if (Command.size() == 1)
    {
        bist.nTestsMask = (uint32_t)XN_BIST_ALL;
    }
    else if (Command.size() == 2 && Command[1] == "help")
    {
        bShowUsage = true;
    }
    else
    {
        for (XnUInt32 i = 1; i < Command.size(); ++i)
        {
            bool bFound = false;

            // search for this test
            for (int j = 0; j < nBistTests; ++j)
            {
                if (sBistTests[j] == Command[i])
                {
                    bist.nTestsMask |= (1 << j);
                    bFound = true;
                    break;
                }
            }

            if (!bFound)
            {
                if (Command[i] == "all")
                {
                    bist.nTestsMask = (uint32_t)XN_BIST_ALL;
                }
                else
                {
                    cout << "Unknown test: " << Command[i] << endl;
                    bShowUsage = true;
                    break;
                }
            }
        }
    }

    if (bShowUsage)
    {
        cout << "Usage: " << Command[0] << " [test_list]" << endl;
        cout << "where test_list can be one or more of:" << endl;
        for (int i = 0; i < nBistTests; ++i)
        {
            cout << "\t" << sBistTests[i] << endl;
        }
        return true;
    }

    openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_BIST, bist);
    if (rc != openni::STATUS_OK)
    {
        printf("Error: %s\n", openni::OpenNI::getExtendedError());
        return true;
    }

    if (bist.nFailures == 0)
    {
        cout << "BIST passed." << endl;
    }
    else
    {
        cout << "BIST failed. Returning code: " << bist.nFailures << endl;
        cout << "The following tests have failed:" << endl;
        for (int i = 0; i < nBistErrors; ++i)
        {
            int nMask = 1 << i;
            if ((bist.nFailures & nMask) != 0)
            {
                cout << "\t" << sBistErrors[i] << endl;
            }
        }
    }

    return true;
}

bool CalibrateTec(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() < 3)
    {
        cout << "Usage: " << Command[0] << " " << Command[1] << " <set point>" << endl;
        return true;
    }

    int nSetPoint;

    if (!atoi2(Command[2].c_str(), (int*)&nSetPoint))
    {
        printf("Can't understand %s as set point\n", Command[2].c_str());
        return true;
    }

    if (nSetPoint > 0xFFFF)
    {
        printf("Set point can't fit in a 16-bit integer!\n");
        return true;
    }

    openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_TEC_SET_POINT, (XnUInt64)nSetPoint);
    if (rc != openni::STATUS_OK)
    {
        printf("%s\n", openni::OpenNI::getExtendedError());
    }

    return true;
}

bool GetTecData(openni::Device& Device, vector<string>& /*Command*/)
{
    XnTecData TecData;

    openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_TEC_STATUS, &TecData);
    if (rc != openni::STATUS_OK)
    {
        printf("%s\n", openni::OpenNI::getExtendedError());
    }
    else
    {
        printf("SetPointVoltage: %hd\nCompensationVoltage: %hd\nDutyCycle: %hd\nHeatMode: %hd\nProportionalError: %d\nIntegralError: %d\nDeriativeError: %d\nScanMode: %hd\n",
            TecData.m_SetPointVoltage, TecData.m_CompensationVoltage, TecData.m_TecDutyCycle,
            TecData.m_HeatMode, TecData.m_ProportionalError, TecData.m_IntegralError,
            TecData.m_DerivativeError, TecData.m_ScanMode);
    }

    return true;
}

bool GetTecFastConvergenceData(openni::Device& Device, vector<string>& /*Command*/)
{
    XnTecFastConvergenceData TecData;
    XnFloat     SetPointTemperature;
    XnFloat     MeasuredTemperature;
    XnFloat     ErrorTemperature;

    openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_TEC_FAST_CONVERGENCE_STATUS, &TecData);
    if (rc != openni::STATUS_OK)
    {
        printf("%s\n", openni::OpenNI::getExtendedError());
    }
    else
    {
        SetPointTemperature = TecData.m_SetPointTemperature;
        MeasuredTemperature = TecData.m_MeasuredTemperature;
        /* calculate error in temperature */
        ErrorTemperature =
            (XnFloat)TecData.m_SetPointTemperature - (XnFloat)TecData.m_MeasuredTemperature;

        /* scale back temperature values, as they are given scaled by factor
        of 100 (for precision) */
        SetPointTemperature = SetPointTemperature / 100;
        MeasuredTemperature = MeasuredTemperature / 100;
        ErrorTemperature = ErrorTemperature / 100;

        printf("SetPointTemperature: %f\nMeasuredTemperature: %f\nError: %f\nDutyCyclePercents: %hd\nHeatMode: %hd\nProportionalError: %d\nIntegralError: %d\nDeriativeError: %d\nScanMode: %hd\nTemperatureRange: %hd\n",
            SetPointTemperature, MeasuredTemperature, ErrorTemperature, TecData.m_TecDutyCycle,
            TecData.m_HeatMode, TecData.m_ProportionalError, TecData.m_IntegralError,
            TecData.m_DerivativeError, TecData.m_ScanMode, TecData.m_TemperatureRange);
    }

    return true;
}

bool Tec(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        if (Command[1] == "calib")
        {
            return CalibrateTec(Device, Command);
        }
        else if (Command[1] == "get")
        {
            return GetTecData(Device, Command);
        }
    }

    cout << "Usage: " << Command[0] << " <calib/get> ..." << endl;
    return true;
}

bool TecFC(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        if (Command[1] == "calib")
        {
            return CalibrateTec(Device, Command);
        }
        else if (Command[1] == "get")
        {
            return GetTecFastConvergenceData(Device, Command);
        }
    }

    cout << "Usage: " << Command[0] << " <calib/get> ..." << endl;
    return true;
}


bool CalibrateEmitter(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() < 3)
    {
        cout << "Usage: " << Command[0] << " " << Command[1] << " <set point>" << endl;
        return true;
    }

    int nSetPoint;

    if (!atoi2(Command[2].c_str(), (int*)&nSetPoint))
    {
        printf("Can't understand %s as set point\n", Command[2].c_str());
        return true;
    }

    if (nSetPoint > 0xFFFF)
    {
        printf("Set point can't fit in a 16-bit integer!\n");
        return true;
    }

    openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_EMITTER_SET_POINT, (XnUInt64)nSetPoint);
    if (rc != openni::STATUS_OK)
    {
        printf("%s\n", openni::OpenNI::getExtendedError());
    }

    return true;
}

bool GetEmitterData(openni::Device& Device, vector<string>& /*Command*/)
{
    XnEmitterData EmitterData;

    openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_EMITTER_STATUS, &EmitterData);
    if (rc != openni::STATUS_OK)
    {
        printf("%s\n", openni::OpenNI::getExtendedError());
    }
    else
    {
        printf("State: %hd\n", EmitterData.m_State);
        printf("SetPointVoltage: %hd\n", EmitterData.m_SetPointVoltage);
        printf("SetPointClocks: %hd\n", EmitterData.m_SetPointClocks);
        printf("PD Reading: %hd\n", EmitterData.m_PD_Reading);
        printf("EmitterSet: %hd\n", EmitterData.m_EmitterSet);
        printf("SettingLogic: %hd\n", EmitterData.m_EmitterSettingLogic);
        printf("LightMeasureLogic: %hd\n", EmitterData.m_LightMeasureLogic);
        printf("APC Enabled: %hd\n", EmitterData.m_IsAPCEnabled);
        printf("StepSize: %hd\n", EmitterData.m_EmitterSetStepSize);

        if (g_DeviceVersion.FWVer < XN_SENSOR_FW_VER_5_3)
        {
            printf("Tolerance: %hd\n", EmitterData.m_ApcTolerance);
        }
        else
        {
            printf("SubClocking: %hd\n", EmitterData.m_SubClocking);
            printf("Precision: %hd\n", EmitterData.m_Precision);
        }
    }

    return true;
}

bool Emitter(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        if (Command[1] == "calib")
        {
            return CalibrateEmitter(Device, Command);
        }
        else if (Command[1] == "get")
        {
            return GetEmitterData(Device, Command);
        }
        else if (Command[1] == "on")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_EMITTER_STATE, (XnUInt64)TRUE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
        else if (Command[1] == "off")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_EMITTER_STATE, (XnUInt64)FALSE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
    }

    cout << "Usage: " << Command[0] << " <calib|get|on|off> ..." << endl;
    return true;
}

bool IrfloodSwitch(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        if (Command[1] == "on")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_IRFLOOD_STATE, (XnUInt64)TRUE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
        else if (Command[1] == "off")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_IRFLOOD_STATE, (XnUInt64)FALSE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
        else if (Command[1] == "get")
        {
            XnUInt32 unValue;
            openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_IRFLOOD_STATE, &unValue);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
                return true;
            }
            printf("Ir flood %s\n", unValue != 0 ? "on" : "off");
            return true;
        }
    }

    cout << "Usage: " << Command[0] << " <get|on|off> ..." << endl;
    return true;
}

bool IrfloodLevel(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        if (Command[1] == "get")
        {
            XnUInt32 unValue;
            openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_IRFLOOD_LEVEL, &unValue);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
                return true;
            }
            printf("Ir flood Level %u\n", unValue);
            return true;
        }
        else if (Command[1] == "set")
        {
            const char* c_pInput = Command[2].c_str();
            if (NULL == c_pInput)
            {
                printf("Input Null\n");
                return true;
            }

            int nValue = 0;
            if (!atoi2(c_pInput, &nValue))
            {
                printf("Input Error\n");
                return true;
            }

            if (nValue < 0)
            {
                printf("Can`t input negative num\n");
                return false;
            }

            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_IRFLOOD_LEVEL, (XnUInt64)nValue);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
                return true;
            }
            return true;
        }
    }
    cout << "Usage: " << Command[0] << " <get | set 'positive num' > ..." << endl;
    return true;
}

bool LdpEnable(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        if (Command[1] == "on")
        {
            openni::Status rc = Device.setProperty(OBEXTENSION_ID_LDP_EN, (XnUInt32)TRUE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
        else if (Command[1] == "off")
        {
            openni::Status rc = Device.setProperty(OBEXTENSION_ID_LDP_EN, (XnUInt32)FALSE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
    }

    cout << "Usage: " << Command[0] << " <calib|get|on|off> ..." << endl;
    return true;
}

bool ProjectorFault(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() < 3)
    {
        cout << "Usage: " << Command[0] << " <min> <max>" << endl;
        return true;
    }

    int nMin, nMax;
    if (!atoi2(Command[1].c_str(), (int*)&nMin))
    {
        printf("Can't understand %s as min\n", Command[1].c_str());
        return true;
    }

    if (!atoi2(Command[2].c_str(), (int*)&nMax))
    {
        printf("Can't understand %s as max\n", Command[2].c_str());
        return true;
    }

    XnProjectorFaultData ProjectorFaultData;
    ProjectorFaultData.nMinThreshold = (uint16_t)nMin;
    ProjectorFaultData.nMaxThreshold = (uint16_t)nMax;

    openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_PROJECTOR_FAULT, ProjectorFaultData);
    if (rc != openni::STATUS_OK)
    {
        printf("Error: %s\n", openni::OpenNI::getExtendedError());
        return true;
    }

    if (ProjectorFaultData.bProjectorFaultEvent)
        printf("ProjectorFault event occurred!\n");
    else
        printf("ProjectorFault OK.\n");

    return true;
}

bool StartReadData(openni::Device& Device, vector<string>& Command)
{
    XnSensorUsbInterface usbInterface = XN_SENSOR_USB_INTERFACE_DEFAULT;

    if (Command.size() > 1)
    {
        if (xnOSStrCaseCmp(Command[1].c_str(), "bulk") == 0)
        {
            usbInterface = XN_SENSOR_USB_INTERFACE_BULK_ENDPOINTS;
        }
        else if (xnOSStrCaseCmp(Command[1].c_str(), "iso") == 0)
        {
            usbInterface = XN_SENSOR_USB_INTERFACE_ISO_ENDPOINTS;
        }
        else
        {
            cout << "Usage: " << Command[0] << " [bulk/iso]" << endl;
            return true;
        }
    }

    openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_USB_INTERFACE, (XnUInt64)usbInterface);
    if (rc != openni::STATUS_OK)
    {
        printf("Can't set USB interface: %s\n", openni::OpenNI::getExtendedError());
        return true;
    }

    if (!g_depthStream.isValid())
    {
        // create stream
        rc = g_depthStream.create(Device, openni::SENSOR_DEPTH);
        if (rc != openni::STATUS_OK)
        {
            printf("Can't create depth stream: %s\n", openni::OpenNI::getExtendedError());
            return true;
        }
    }

    if (!g_colorStream.isValid())
    {
        rc = g_colorStream.create(Device, openni::SENSOR_COLOR);
        if (rc != openni::STATUS_OK)
        {
            printf("Can't create image stream: %s\n", openni::OpenNI::getExtendedError());
            return true;
        }
    }

    // and cause reading to take place (without start streaming)
    rc = g_depthStream.setProperty(XN_STREAM_PROPERTY_ACTUAL_READ_DATA, TRUE);
    if (rc != openni::STATUS_OK)
    {
        printf("Can't start depth endpoint: %s\n", openni::OpenNI::getExtendedError());
        return true;
    }

    rc = g_colorStream.setProperty(XN_STREAM_PROPERTY_ACTUAL_READ_DATA, TRUE);
    if (rc != openni::STATUS_OK)
    {
        printf("Can't start image endpoint: %s\n", openni::OpenNI::getExtendedError());
        return true;
    }

    printf("Endpoints are now open.\n");

    return true;
}

bool IrGainSet(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {

        int nGainValue;
        if (!atoi2(Command[1].c_str(), &nGainValue))
        {
            printf("Can't understand '%s' as gain", Command[1].c_str());
            return true;
        }

        openni::Status rc = Device.setProperty(OBEXTENSION_ID_IR_GAIN, (XnUInt32)nGainValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <IrGain gainValue> ..." << endl;
    return true;
}

//get IrGain
bool GetIrGain(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nGainValue = 0;
        openni::Status rc = Device.getProperty(OBEXTENSION_ID_IR_GAIN, &nGainValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("irGain: %d\n", nGainValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <IrGain gainValue> ..." << endl;
    return true;
}

//Set IrTemp
bool SetCalIrTemp(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {

        double nValue = (double)atof(Command[1].c_str());
        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_CAL_IR_TEMP, (XnDouble)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <Cal IrTemp temperautreValue> ..." << endl;
    return true;
}

//get IrTemp
bool GetCalIrTemp(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnDouble nIrValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_CAL_IR_TEMP, &nIrValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Cal irTemp: %f\n", nIrValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <Cal IrTemp temperautreValue> ..." << endl;
    return true;
}

//Set LdmpTemp
bool SetCalLdmpTemp(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        double nValue = (double)atof(Command[1].c_str());
        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_CAL_LDMP_TEMP, (XnDouble)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <Cal LdmpTemp temperautreValue> ..." << endl;
    return true;
}

//get IrTemp
bool GetCalLdmpTemp(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnDouble nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_CAL_LDMP_TEMP, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Cal LdmpTemp: %f\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <Cal LdmpTemp temperautreValue> ..." << endl;
    return true;
}

//get real time ir temperature
bool GetRtIrTemp(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnDouble nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_RT_IR_TEMP, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Real time ir temperature: %f\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <Real time ir temperautre Value> ..." << endl;
    return true;
}

//get Ldmp real time temperature
bool GetRtLdmpTemp(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnDouble nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_RT_LDMP_TEMP, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Real time ldmp temperature: %f\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <Real time ldmp temperautre Value> ..." << endl;
    return true;
}

//set ir temperature compensation coefficient
bool SetIrTemperatureCo(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        double nValue = (double)atof(Command[1].c_str());
        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_IR_TEMP_COMP_CO, (XnDouble)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <Ir tempco Value> ..." << endl;
    return true;
}

//get ir temperature compensation coefficient
bool GetIrTemperatureCo(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnDouble nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_IR_TEMP_COMP_CO, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Ir tempco: %f\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <Ir tempco Value> ..." << endl;
    return true;
}

//set ldmp temperature compensation coefficient
bool SetLdmpTemperatureCo(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        double nValue = (double)atof(Command[1].c_str());
        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_LDMP_TEMP_COMP_CO, (XnDouble)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <LDMP tempco Value> ..." << endl;
    return true;
}

//get ldmp temperature compensation coefficient
bool GetLdmpTemperatureCo(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnDouble nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_LDMP_TEMP_COMP_CO, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("LDMP tempco: %f\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <LDMP tempco Value> ..." << endl;
    return true;
}

//Temperature comp
bool TempCompEnable(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        if (Command[1] == "on")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_TEMP_COMP, (XnUInt32)TRUE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
        else if (Command[1] == "off")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_TEMP_COMP, (XnUInt32)FALSE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
    }

    cout << "Usage: " << Command[0] << " <calib|get|on|off> ..." << endl;
    return true;
}

bool GetTempCompStatus(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_TEMP_COMP, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("temperature compensation status: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " < get temperature compensation status> ..." << endl;
    return true;
}

//get serial number
bool GetSerialNumber(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        char serNumber[16] = { 0 };
        int dataSize = sizeof(serNumber);
        openni::Status rc = Device.getProperty(OBEXTENSION_ID_SERIALNUMBER, &serNumber, &dataSize);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("serial number: %s\n", serNumber);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <serial number> ..." << endl;
    return true;
}

//get device type
bool GetDeviceType(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        char deviceType[32] = { 0 };
        int dataSize = sizeof(deviceType);
        openni::Status rc = Device.getProperty(OBEXTENSION_ID_DEVICETYPE, &deviceType, &dataSize);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("device type: %s\n", deviceType);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <device type> ..." << endl;
    return true;
}

bool IrExpSet(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {

        int nValue;
        if (!atoi2(Command[1].c_str(), &nValue))
        {
            printf("Can't understand '%s' as exp", Command[1].c_str());
            return true;
        }

        openni::Status rc = Device.setProperty(OBEXTENSION_ID_IR_EXP, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <Irexp expValue> ..." << endl;
    return true;
}

//get IrExp
bool GetIrExp(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(OBEXTENSION_ID_IR_EXP, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("irExp: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <Irexp expValue> ..." << endl;
    return true;
}

//
bool AdoChangeSensor(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        if (Command[1] == "on")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_SENSOR_CHANGE, (XnUInt64)TRUE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
        else if (Command[1] == "off")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_SENSOR_CHANGE, (XnUInt64)FALSE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
    }

    cout << "Usage: " << Command[0] << " <on|off> ..." << endl;
    return true;
}


bool GetEccVerifyData(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        OBEccVerify obEccVerify;

        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_PUBLIC_KEY, &obEccVerify);
        if (rc != openni::STATUS_OK)
        {
            printf("rc =%d,%s\n", rc, openni::OpenNI::getExtendedError());
        }
        else
        {

            printf("get batch version string\n");
            char batch_ver[16] = { 0 };
            char publicX[52] = { 0 };
            char publicY[52] = { 0 };

            memcpy(batch_ver, obEccVerify.Batch_Ver, 12);
            memcpy(publicX, obEccVerify.Pub_x, 48);
            memcpy(publicY, obEccVerify.Pub_y, 48);

            printf("%s\n", batch_ver);
            printf("%s\n", publicX);
            printf("%s\n", publicY);
            printf("\n");
        }
    }

    cout << "Usage: " << Command[0] << " <GetEccVerifyData > ..." << endl;

    return true;
}

bool SetEccVerifyData(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        OBEccVerify obEccVerify;
        memset(&obEccVerify, 0, sizeof(OBEccVerify));
        std::string publicX = "D0114B2CB82E704FDCF3C102ABB81108170A577EDD0C38CA";
        int nPublicX = (int)publicX.size();
        std::string publicY = "31D3C7D0681DE85C05B4F52AB8C5256E6AC95254C03CDA69";

        int nPublicY = (int)publicY.size();
        std::string batchVersion = "201803220001";
        int nBatchVersion = (int)batchVersion.size();

        printf("nPublicX=%d,nPublicY=%d,nBatchVersion=%d\n", nPublicX, nPublicY, nBatchVersion);

        memcpy(obEccVerify.Pub_x, publicX.c_str(), nPublicX);
        memcpy(obEccVerify.Pub_y, publicY.c_str(), nPublicY);
        memcpy(obEccVerify.Batch_Ver, batchVersion.c_str(), nBatchVersion);



        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_PUBLIC_KEY, obEccVerify);
        if (rc != openni::STATUS_OK)
        {
            printf("%s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("set EccVerify success\n");
        }

    }

    cout << "Usage: " << Command[0] << " <SetEccVerifyData > ..." << endl;
    return true;
}


//
bool GetRandomString(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        OBEccInit obEccInit;

        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_RANDOM_STRING, &obEccInit);
        if (rc != openni::STATUS_OK)
        {
            printf("rc=%d,%s\n", rc, openni::OpenNI::getExtendedError());
        }
        else
        {
            //printf("get batch version string\n");
            //for (int i = 0; i < 12; i++)
            //{
            //	printf("%x", obEccInit.Batch_Ver[i]);
            //}
            //printf("\n");

            char batch_ver[16] = { 0 };
            char radromString[52] = { 0 };

            memcpy(batch_ver, obEccInit.Batch_Ver, 12);
            memcpy(radromString, obEccInit.RandomStr, 48);

            printf("%s\n", batch_ver);
            printf("%s\n", radromString);


        }
    }

    cout << "Usage: " << Command[0] << " <GetRandomString > ..." << endl;
    return true;
}

bool SetRSKey(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        OBEccRSKey obEccRsKey;
        memset(&obEccRsKey, 0, sizeof(obEccRsKey));
        std::string RsKeyX = "80FA4EEDA11BFDCCF9CF04C71F420193E9444DB9A02541BA";
        int nPublicX = (int)RsKeyX.size();
        std::string RsKeyY = "0D32F0DA3371600A94FB503271CED2132C7C82B07E5D686F";

        int nPublicY = (int)RsKeyY.size();


        printf("RsKeyX=%d,RsKeyY=%d\n", nPublicX, nPublicY);

        memcpy(obEccRsKey.R_key, RsKeyX.c_str(), nPublicX);
        memcpy(obEccRsKey.S_key, RsKeyY.c_str(), nPublicY);

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_RS_KEY, obEccRsKey);
        if (rc != openni::STATUS_OK)
        {
            printf("%s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("set SetRSKey success\n");
        }
    }

    cout << "Usage: " << Command[0] << " <SetRSKey > ..." << endl;
    return true;
}

long getFileSize(const char* filename)
{
    FILE* fp = fopen(filename, "rb");
    if (NULL != fp)
    {
        fseek(fp, 0, SEEK_END);
        long size = ftell(fp);
        fclose(fp);
        return size;
    }

    return -1;
}

bool readFile(const char* path, uint8_t* buf, uint32_t size)
{
    FILE* fp = fopen(path, "rb");
    if (NULL == fp)
        return false;

    uint32_t readedSize = (uint32_t)fread(buf, sizeof(uint8_t), size, fp);
    if (readedSize != size)
    {
        fclose(fp);
        return false;
    }

    fclose(fp);
    return true;
}

//read bin end
bool updateAppCode(char * filename, openni::Device& Device) {
    if (filename == NULL)
        return false;
    uint8_t *buf = NULL;
    uint32_t nsize;
    nsize = getFileSize(filename);
    buf = (unsigned char *)malloc(sizeof(unsigned char)*nsize);
    bool bRet = readFile(filename, buf, nsize);
    if (!bRet)
    {
        return false;
    }

    openni::Status rc = Device.setProperty(openni::OBEXTENSION_ID_UPDATE_FIRMWARE, buf, nsize);
    if (rc != openni::STATUS_OK)
    {
        printf("%s,%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
        free(buf);
        return false;
    }
    else
    {
        printf("upload firmware success\n");
        free(buf);
        return true;
    }

}

bool updateFirmware(openni::Device& Device, vector<string>& Command) {
    if (Command.size() == 1)
    {
        std::string strBin = "D:\\Firmware\\Mx6000_Canglong_2419.bin";
        updateAppCode((char *)strBin.c_str(), Device);
        return true;
    }
    cout << "Usage: " << Command[0] << " <updateUsbAppCode> ..." << endl;
    return true;
}


bool KeepAlive(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_LASER_SECURE_KEEPALIVE, NULL, 0);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("keep alive success\n");
        }
    }

    cout << "Usage: " << Command[0] << " <KeepAlive > ..." << endl;
    return true;
}

bool IsSupportLaserSecure(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_IS_SUPPORT_LASER_SECURE, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Is support laser secure: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <Irexp expValue> ..." << endl;
    return true;
}

//open/close laser secure
bool LaserSecureEnable(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        if (Command[1] == "on")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_LASER_SECURE_STATUS, (XnUInt32)TRUE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
        else if (Command[1] == "off")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_LASER_SECURE_STATUS, (XnUInt32)FALSE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
    }

    cout << "Usage: " << Command[0] << " <on|off> ..." << endl;
    return true;
}

//
bool GetLaserSecureStatus(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_LASER_SECURE_STATUS, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("laser secure status: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " < get laser secure status> ..." << endl;
    return true;
}

bool LaserCurrentSet(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {

        int nValue;
        if (!atoi2(Command[1].c_str(), &nValue))
        {
            printf("Can't understand '%s' as laser current", Command[1].c_str());
            return true;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_LASER_CURRENT, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <Laser Current value> ..." << endl;
    return true;
}

//get laser current
bool GetLaserCurrent(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_LASER_CURRENT, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("laserCurrent: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <Laser Current Value> ..." << endl;
    return true;
}

bool SoftReset(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_SOFT_RESET, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("soft reset success\n");
        }
        return true;
    }

    cout << "Usage: " << Command[0] << " <soft reset> ..." << endl;
    return true;
}
bool SwitchIr(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        if (Command[1] == "left")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_SWITCH_IR, (XnUInt32)FALSE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
        else if (Command[1] == "right")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_SWITCH_IR, (XnUInt32)TRUE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
    }

    cout << "Usage: " << Command[0] << " <left|right> ..." << endl;
    return true;
}

bool SetCameraParams(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        OBCameraParams cameraParam;
        cameraParam.l_intr_p[0] = (float)577.318970;
        cameraParam.l_intr_p[1] = (float)577.318970;
        cameraParam.l_intr_p[2] = (float)308.729004;
        cameraParam.l_intr_p[3] = (float)269.143005;

        cameraParam.r_intr_p[0] = (float)517.318970;
        cameraParam.r_intr_p[1] = (float) 517.318970;
        cameraParam.r_intr_p[2] = (float)305.729004;
        cameraParam.r_intr_p[3] = (float)250.143005;

        for (int i = 0; i < 9; i++)
        {
            cameraParam.r2l_r[i] = (float)9.143005;
        }

        for (int i = 0; i < 3; i++)
        {
            cameraParam.r2l_t[i] = (float)10.143005;
        }

        for (int i = 0; i < 5; i++)
        {
            cameraParam.l_k[i] = (float)11.143005;
        }

        for (int i = 0; i < 5; i++)
        {
            cameraParam.r_k[i] = (float)12.143005;
        }


        int dataSize = sizeof(cameraParam);

        openni::Status rc = Device.setProperty(openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t*)&cameraParam, dataSize);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        printf("set cameraparams success\n");
        return true;
    }

    cout << "Usage: " << Command[0] << " <set ObCameraParam> ..." << endl;
    return true;
}

bool getCameraParams(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        OBCameraParams cameraParam;
        int dataSize = sizeof(cameraParam);
        memset(&cameraParam, 0, sizeof(cameraParam));
        openni::Status rc = Device.getProperty(openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t *)&cameraParam, &dataSize);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        printf("Calibration resolution: %d x %d\n", cameraParam.width, cameraParam.height);

        for (int i = 0; i < 4; i++)
        {
            printf("l_intr_p[%d] = %lf\n", i, cameraParam.l_intr_p[i]);
        }

        for (int i = 0; i < 4; i++)
        {
            printf("r_intr_p[%d] = %lf\n", i, cameraParam.r_intr_p[i]);
        }


        for (int i = 0; i < 9; i++)
        {
            printf("r2l_r[%d] = %lf\n", i, cameraParam.r2l_r[i]);
        }

        for (int i = 0; i < 3; i++)
        {
            printf("r2l_t[%d] = %lf\n", i, cameraParam.r2l_t[i]);
        }

        for (int i = 0; i < 5; i++)
        {
            printf("l_k[%d] = %lf\n", i, cameraParam.l_k[i]);
        }

        for (int i = 0; i < 5; i++)
        {
            printf("r_k[%d] = %lf\n", i, cameraParam.r_k[i]);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <set ObCameraParam> ..." << endl;
    return true;
}

bool SetRgbAeMode(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 3)
    {
        XnRgbAeMode ObRgbAeMode;
        int nAeMode, nAeTarget;
        if (!atoi2(Command[1].c_str(), &nAeMode))
        {
            printf("can't understand %s as AeMode\n", Command[1].c_str());
            return true;
        }

        if (!atoi2(Command[2].c_str(), &nAeTarget))
        {
            printf("can't understand %s as AeTarget\n", Command[2].c_str());
            return true;
        }

        ObRgbAeMode.nAeMode = (uint16_t)nAeMode;
        ObRgbAeMode.nAeTarget = (uint16_t)nAeTarget;
        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_RGB_AE_MODE, ObRgbAeMode);

        if (rc != openni::STATUS_OK)
        {
            printf("%s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("set SetRgbAeMode success\n");
        }
    }

    cout << "Usage: " << Command[0] << " <SetRgbAeMode > <AeMode> <AeTarget>" << endl;
    return true;
}

//Set DepthOptim  Param
//bool SetDepthOptimParam(openni::Device& Device, vector<string>& Command)
//{
//    if (Command.size() == 4)
//    {
//        XnDepthOptimizationParam depthOptimParam;
//        double nParam1, nParam2, nParam3;
//
//        nParam1 = atof(Command[1].c_str());
//        nParam2 = atof(Command[2].c_str());
//        nParam3 = atof(Command[3].c_str());
//
//        depthOptimParam.nParam1 = (double)nParam1;
//        depthOptimParam.nParam2 = (double)nParam2;
//        depthOptimParam.nParam3 = (double)nParam3;
//
//        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_DEPTH_OPTIM_PARAM, depthOptimParam);
//        if (rc != openni::STATUS_OK)
//        {
//            printf("%s\n", openni::OpenNI::getExtendedError());
//        }
//        else
//        {
//            printf("Set depthOptimParam success\n");
//        }
//    }
//
//    cout << "Usage: " << Command[0] << " <SetDepthOptimParam >..." << endl;
//
//    return true;
//}
//
//bool GetDepthOptimParam(openni::Device& Device, vector<string>& Command)
//{
//    if (Command.size() == 1)
//    {
//        XnDepthOptimizationParam obDepthOptimParam;
//
//        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_DEPTH_OPTIM_PARAM, &obDepthOptimParam);
//        if (rc != openni::STATUS_OK)
//        {
//            printf("%s\n", openni::OpenNI::getExtendedError());
//        }
//        else
//        {
//
//            printf("Get depthOptimParam  Param1=%lf, Param2=%lf, Param3=%lf\n", obDepthOptimParam.nParam1, obDepthOptimParam.nParam2, obDepthOptimParam.nParam3);
//        }
//    }
//
//    cout << "Usage: " << Command[0] << " <GetDepthOptimParam > ..." << endl;
//
//    return true;
//}
//
////Depth optim enable
//bool DepthOptimEnable(openni::Device& Device, vector<string>& Command)
//{
//    if (Command.size() > 1)
//    {
//        if (Command[1] == "on")
//        {
//            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_DEPTH_OPTIM_STATE, (XnUInt32)TRUE);
//            if (rc != openni::STATUS_OK)
//            {
//                printf("Error: %s\n", openni::OpenNI::getExtendedError());
//            }
//            return true;
//        }
//        else if (Command[1] == "off")
//        {
//            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_DEPTH_OPTIM_STATE, (XnUInt32)FALSE);
//            if (rc != openni::STATUS_OK)
//            {
//                printf("Error: %s\n", openni::OpenNI::getExtendedError());
//            }
//            return true;
//        }
//    }
//
//    cout << "Usage: " << Command[0] << " <calib|get|on|off> ..." << endl;
//    return true;
//}
//
//bool GetDepthOptimStatus(openni::Device& Device, vector<string>& Command)
//{
//    if (Command.size() == 1)
//    {
//        XnUInt32 nValue = 0;
//        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_DEPTH_OPTIM_STATE, &nValue);
//        if (rc != openni::STATUS_OK)
//        {
//            printf("Error: %s\n", openni::OpenNI::getExtendedError());
//        }
//        else
//        {
//            printf("Depth optimization status: %d\n", nValue);
//        }
//
//        return true;
//    }
//
//    cout << "Usage: " << Command[0] << " < get depth optimization status> ..." << endl;
//    return true;
//}

bool GetRgbAeMode(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnRgbAeMode obRgbAeMode;

        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_RGB_AE_MODE, &obRgbAeMode);
        if (rc != openni::STATUS_OK)
        {
            printf("%s\n", openni::OpenNI::getExtendedError());
        }
        else
        {

            printf("nRgbAeMode =%d,nRgbAeTarget =%d\n", obRgbAeMode.nAeMode, obRgbAeMode.nAeTarget);
        }
    }

    cout << "Usage: " << Command[0] << " <GetRgbAeMode > ..." << endl;

    return true;
}

bool readFlashForTest(openni::Device& Device, vector<string>& Command)
{
    //read flash to verify
    if (Command.size() == 4)
    {

        std::string strFile = Command[1].c_str();
        if (strFile.empty() || strFile.find_last_of("bin") <= 0)
        {
            printf("Error: Please drag in a bin file!\n");
            return true;
        }


        int nOffset;
        int nSize;
        if (!atoi2(Command[2].c_str(), (int*)&nOffset))
        {
            printf("Can't understand %s as \n", Command[2].c_str());
            return true;
        }

        if (!atoi2(Command[3].c_str(), (int*)&nSize))
        {
            printf("Can't understand %s as \n", Command[3].c_str());
            return true;
        }

        XnParamFlashData readFlash;
        readFlash.pData = (unsigned char *)malloc(sizeof(unsigned char)*nSize);
        readFlash.nOffset = nOffset;
        readFlash.nSize = nSize;

        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_UPDATE_FIRMWARE_FLASH_CHUNK, &readFlash);
        if (rc != openni::STATUS_OK)
        {
            printf("%s,%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
            free(readFlash.pData);
            return false;
        }
        else
        {
            //save file
            xnOSSaveFile(strFile.c_str(), readFlash.pData, readFlash.nSize);
            free(readFlash.pData);
            printf("read flash succ\n");
            return true;

        }
    }


    cout << "Usage: " << Command[0] << " <222222222 > ..." << endl;
    return true;

}

bool UpdateFirewareFlashChunk(char * filename, openni::Device& Device)
{
    if (filename == NULL)
    {
        return false;
    }

    uint32_t nSize = getFileSize(filename);
    if (nSize % 2 != 0)
    {
        nSize += 1;
    }

    XnParamFlashData ParamFlash;
    ParamFlash.pData = (unsigned char *)malloc(sizeof(unsigned char)*nSize);
    bool bRet = readFile(filename, ParamFlash.pData, nSize);
    if (!bRet)
    {
        free(ParamFlash.pData);
        return false;
    }


    ParamFlash.nOffset = 0x20000;
    ParamFlash.nSize = nSize;
    int nTotalDataSize = sizeof(XnParamFlashData) + nSize - sizeof(ParamFlash.pData);
    openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_UPDATE_FIRMWARE_FLASH_CHUNK, &ParamFlash, nTotalDataSize);
    if (rc != openni::STATUS_OK)
    {
        printf("%s,%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
        free(ParamFlash.pData);
        return false;
    }
    else
    {
        printf("upload firmware success\n");

        //read flash to verify
        XnParamFlashData readFlash;
        readFlash.pData = (unsigned char *)malloc(sizeof(unsigned char)*nSize);
        readFlash.nOffset = 0x20000;
        readFlash.nSize = nSize;

        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_UPDATE_FIRMWARE_FLASH_CHUNK, &readFlash);
        if (rc != openni::STATUS_OK)
        {
            printf("%s,%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
            free(ParamFlash.pData);
            free(readFlash.pData);
            return false;
        }
        else
        {
            //verify data
            bool status = false;
            for (uint32_t i = 0; i < nSize; i++)
            {
                if (ParamFlash.pData[i] != readFlash.pData[i])
                {
                    status = true;
                    break;
                }
            }

            free(ParamFlash.pData);
            free(readFlash.pData);

            if (status)
            {
                printf("read flash verify failed\n");
                return false;
            }
            else
            {
                printf("read flash verify succ\n");
                return true;
            }
        }

    }
}

bool updateFirmwareFlash(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        std::string strBin = "Mx6000_LunaP2_2407.bin";
        UpdateFirewareFlashChunk((char *)strBin.c_str(), Device);
        return true;
    }
    cout << "Usage: " << Command[0] << " <updateUsbAppCode> ..." << endl;
    return true;
}

bool LdpEnable_Register(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        if (Command[1] == "on")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_LDP_ENABLE, (XnUInt32)TRUE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
        else if (Command[1] == "off")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_LDP_ENABLE, (XnUInt32)FALSE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
    }

    cout << "Usage: " << Command[0] << " <on|off> ..." << endl;
    return true;
}

bool GetLdpEnable_Register(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_LDP_ENABLE, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("ldp enable register status: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " < get ldp enable register status> ..." << endl;
    return true;
}

bool GetEmitterEnable_Register(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_EMITTER_STATE_V1, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("emitter enable register status: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " < get emitter enable register status> ..." << endl;
    return true;
}

bool GetLdpScale(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_LDP_SCALE, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("ldp scale : %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " < get ldp scale status> ..." << endl;
    return true;
}

bool SetLdpScale(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        int nValue;
        if (!atoi2(Command[1].c_str(), &nValue))
        {
            printf("Can't understand '%s' as gain", Command[1].c_str());
            return true;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_LDP_SCALE, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <SetLdpScale ScaleValue> ..." << endl;
    return true;
}

bool GetLdpThresUp(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_LDP_THRES_UP, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("ldp thres up value : %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " < get ldp thres up status> ..." << endl;
    return true;
}

bool SetLdpThresUp(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        int nValue;
        if (!atoi2(Command[1].c_str(), &nValue))
        {
            printf("Can't understand '%s' as gain", Command[1].c_str());
            return true;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_LDP_THRES_UP, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <SetLdpThresUp Value> ..." << endl;
    return true;
}

bool GetLdpThresLow(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_LDP_THRES_LOW, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("ldp thres low value : %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " < get ldp thres low status> ..." << endl;
    return true;
}

bool SetLdpThresLow(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        int nValue;
        if (!atoi2(Command[1].c_str(), &nValue))
        {
            printf("Can't understand '%s' as gain", Command[1].c_str());
            return true;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_LDP_THRES_LOW, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <SetLdpThresLow Value> ..." << endl;
    return true;
}

bool GetLdpStatus(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_LDP_STATUS, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("ldp enable register status: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " < get ldp status register status> ..." << endl;
    return true;
}

bool GetLdpNoise(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_LDP_NOIST_VALUE, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("LDP noise value: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " < get ldp status register status> ..." << endl;
    return true;
}

bool SetFirmwareQN(openni::Device& Device, vector<string>& Command)
{
    OBFirmwareQN qN = { 'O', 'B', 'Q', 'N', 0, 0, 0, 0 };
    if (Command.size() > 9) {
        cout << "Usage: " << Command[0] << " QN int bytes ..." << endl;
        return true;
    }

    else if (Command.size() > 1){
        memset(qN.QN, 0, sizeof(qN.QN));
        for (uint16_t i = 1; i < Command.size(); i++) {
            qN.QN[i - 1] = (uint8_t)atoi(Command[i].c_str());
        }
    }

    printf("Write QN: ");
    for (uint32_t i = 0; i < sizeof(qN); i++)
    {
        printf("0x%02x ", qN.QN[i]);
    }
    printf("\n");

    openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_QN_INFO, qN);
    if (rc != openni::STATUS_OK)
    {
        printf("Error: %s\n", openni::OpenNI::getExtendedError());
    }
    else
    {
        printf("Set QN success\n");
    }

    return true;
}

bool GetFirmwareQN(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        OBFirmwareQN qN = { 0 };
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_QN_INFO, &qN);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Get QN success: ");
            for (uint32_t i = 0; i < sizeof(qN); i++)
            {
                printf("0x%02x ", qN.QN[i]);
            }
            printf("\n");
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " < get ldp status register status> ..." << endl;
    return true;
}

bool GetPublicBoardVersion(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        OBPublicBoardVersion publicBoardVersion = { 0 };
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_PUBLIC_BOARD_VERSION, &publicBoardVersion);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("GetPublicBoardVersion success: ");
            for (uint32_t i = 0; i < sizeof(publicBoardVersion); i++)
            {
                printf("%c ", publicBoardVersion.cVersion[i]);
            }
            printf("\n");
        }
    }
    cout << "Usage: " << Command[0] << " < GetPublicBoardVersion > ..." << endl;
    return true;
}

bool VerifyFirmwareQN(openni::Device& Device, vector<string>& Command)
{
    //OBQN0000
    OBFirmwareQN qN = { 'O', 'B', 'Q', 'N', 0, 0, 0, 0 };
    if (Command.size() > 9) {
        cout << "Usage: " << Command[0] << " QN int bytes ..." << endl;
        return true;
    }

    else if (Command.size() > 1) {
        memset(qN.QN, 0, sizeof(qN.QN));
        for (uint16_t i = 1; i < Command.size(); i++) {
            qN.QN[i - 1] = (uint8_t)atoi(Command[i].c_str());
        }
    }

    printf("Verify QN: ");
    for (uint32_t i = 0; i < sizeof(qN); i++)
    {
        printf("0x%02x ", qN.QN[i]);
    }
    printf("\n");

    openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_QN_VERIFY, qN);
    if (rc != openni::STATUS_OK)
    {
        printf("Error: %s\n", openni::OpenNI::getExtendedError());
    }
    else
    {
        printf("Verify QN success\n");
    }
    return true;
}

bool PrintVersionMX6300(openni::Device& Device, vector<string>& /*Command*/)
{
    ObMX6300Version version;
    openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_VERSION_MX6300, &version);
    if (rc != openni::STATUS_OK)
    {
        printf("Error: %s\n", openni::OpenNI::getExtendedError());
    }
    else
    {
        printf("0x");
        uint32_t nSize = sizeof(ObMX6300Version);
        for (uint32_t i = 0; i < nSize; ++i)
        {
            printf("%02x", version.cVersion[nSize - i - 1]);
        }
        printf("\n");
    }
    return true;
}

bool D2CResolutionSet(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {

        int nValue;
        if (!atoi2(Command[1].c_str(), &nValue))
        {
            printf("Can't understand '%s' as Resolution", Command[1].c_str());
            return true;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_D2C_RESOLUTION, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <D2C resolution> ..." << endl;
    return true;
}


bool GetD2CResolution(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_D2C_RESOLUTION, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("resolution: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <D2CResolution value> ..." << endl;
    return true;
}

bool GetUSBSpeed(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_USB_SPEED, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            switch (nValue)
            {
            case XN_USB_LOW_SPEED:
                printf("low speed\n");
                break;
            case XN_USB_FULL_SPEED:
                printf("full speed\n");
                break;
            case XN_USB_HIGH_SPEED:
                printf("high speed\n");
                break;
            case XN_USB_SUPER_SPEED:
                printf("super speed\n");
                break;
            default:
                break;
            }
            printf("USB Speed: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <GetUSBSpeed value> ..." << endl;
    return true;
}

bool SetS1SerialNumber(openni::Device& device, vector<string>& command)
{
    if (command.size() < 2)
    {
        printf("Bad command..., Usage: SetS1SerialNumber <serial number>\n");
        return true;
    }

    OBSerialNumber deviceSN = { 0 };
    std::string input = command[1].c_str();
    uint32_t maxLen = sizeof(deviceSN.SN);
    if (input.size() > maxLen)
    {
        printf("The length of device serial number is too long (%d), it must be limited to %d...", (uint32_t)input.size(), maxLen);
        return true;
    }

    deviceSN.size = (uint32_t)input.size();
    memcpy(deviceSN.SN, input.c_str(), deviceSN.size);
    openni::Status rc = device.setProperty(XN_MODULE_PROPERTY_PRODUCT_SERIAL_NUMBER, deviceSN);
    if (rc != openni::STATUS_OK)
    {
        printf("Error: %s\n", openni::OpenNI::getExtendedError());
        printf("Failed to get device serial number, error (%s)...\n", openni::OpenNI::getExtendedError());
        return true;
    }

    printf("Set device serial number (%s) successfully\n", input.c_str());
    return true;
}

bool GetS1SerialNumber(openni::Device& device, vector<string>& command)
{
    if (command.size() < 1)
    {
        printf("Bad command..., Usage: GetS1SerialNumber\n");
        return true;
    }

    OBSerialNumber deviceSN = { 0 };
    openni::Status rc = device.getProperty(XN_MODULE_PROPERTY_PRODUCT_SERIAL_NUMBER, &deviceSN);
    if (openni::STATUS_OK != rc)
    {
        printf("Failed to get device serial number, error (%s)...\n", openni::OpenNI::getExtendedError());
        return true;
    }

    char sn[64] = { 0 };
    for (uint32_t i = 0; i < deviceSN.size; ++i)
    {
        sprintf(sn + i * 2, "%02x", deviceSN.SN[i]);
    }

    printf("Getting device serial number: %s\n", sn);
    return true;
}

bool SetDevice_ProductNumber(openni::Device& Device, vector<string>& Command)
{
    OBKTProductNumber pN = { 0 };
    memset(pN.PN, 0, sizeof(pN.PN));
    if (Command.size() != 2) {
        cout << "Usage: " << Command[0] << " PN  ..." << endl;
        return true;
    }

    std::string strPN = Command[1].c_str();
    size_t nPNLen = strPN.size();
    uint32_t nObPNLen = sizeof(OBKTProductNumber);
    if (nPNLen > nObPNLen)
    {
        printf("product number len too long,max len is %d\n", nObPNLen);
        return true;
    }

    memcpy(pN.PN, strPN.c_str(), nPNLen);
    openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_PRODUCT_NUMBER, pN);

    if (rc != openni::STATUS_OK)
    {
        printf("Error: %s\n", openni::OpenNI::getExtendedError());
    }
    else
    {
        printf("Set Product PN success\n");
    }


    return true;
}

bool GetDevice_ProductNumber(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        OBKTProductNumber pN = { 0 };
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_PRODUCT_NUMBER, &pN);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Get PN success: ");
            printf("%s\n", pN.PN);
            printf("\n");
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " < get product number> ..." << endl;
    return true;
}

bool SetAeEnable(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 2)
    {
        if (Command[1] == "on")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_AE, (XnUInt64)TRUE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            else
            {
                printf("Open AE success!\n");
            }
            return true;
        }
        else if (Command[1] == "off")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_AE, (XnUInt64)FALSE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            else
            {
                printf("Close AE success!\n");
            }
            return true;
        }
    }

    cout << "Usage: " << Command[0] << " <on|off> ..." << endl;
    return true;
}

bool GetAeEnableStatus(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_AE, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Ae status: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " < get ae status> ..." << endl;
    return true;


}

bool SetMipiTestEnable(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 2)
    {
        if (Command[1] == "on")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_MIPI_TEST, (XnUInt64)TRUE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            else
            {
                printf("Mipi test success!\n");
            }
            return true;
        }
        else if (Command[1] == "off")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_MIPI_TEST, (XnUInt64)FALSE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            else
            {
                printf("mipi test success!\n");
            }
            return true;
        }
    }

    cout << "Usage: " << Command[0] << " <on|off> ..." << endl;
    return true;
}

bool GetMipiTestEnableStatus(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_MIPI_TEST, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("mipi test status: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " < get mipi test status> ..." << endl;
    return true;

}

bool SetCfgSerialProductNumber(openni::Device& Device, vector<string>& Command)
{
    OBCfgSerialProductNumber cfgSn_Pn;
    memset(&cfgSn_Pn, 0, sizeof(OBCfgSerialProductNumber));
    if (Command.size() != 3)
    {
        cout << "Usage: " << Command[0] << " < SerialNumber ProductNumber > ..." << endl;
        return true;
    }

    std::string sn = Command[1].c_str();
    int mSnLen = (int)sn.size();
    if (mSnLen > 12)
    {
        printf("Error: SerialNumber over length 12 \n");
        return true;
    }
    memcpy(cfgSn_Pn.SerialNumber, sn.c_str(), mSnLen);

    std::string pn = Command[2].c_str();
    int mPnLen = (int)pn.size();
    if (mPnLen > 12)
    {
        printf("Error: ProductNumber over length 12 \n");
        return true;
    }

    memcpy(cfgSn_Pn.ProductNumber, pn.c_str(), mPnLen);

    openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_CFG_SNPN, cfgSn_Pn);
    if (rc != XN_STATUS_OK)
    {
        printf("Error: %s\n", openni::OpenNI::getExtendedError());
    }
    else
    {
        printf("Set config SN and PN success!\n");
    }

    return true;
}


bool GetCfgProductNumber(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        char productNumber[16] = { 0 };
        int dataSize = sizeof(productNumber);
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_CFG_PN, &productNumber, &dataSize);
        if (rc != XN_STATUS_OK)


        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Product number: %s\n", productNumber);
        }
    }

    cout << "Usage: " << Command[0] << " <GetCfgPn> ..." << endl;
    return true;
}

bool SetReference(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 2)
    {
        //write bin
        std::string refFile = Command[1].c_str();
        if (refFile.empty() || refFile.find_last_of("bin") <= 0)
        {
            printf("Error: Please drag in a bin file!\n");
            return true;
        }

        uint8_t *buf = NULL;
        uint32_t nSize;
        nSize = getFileSize((char *)refFile.c_str());
        buf = (unsigned char *)malloc(sizeof(unsigned char)*nSize);
        bool bRet = readFile((char *)refFile.c_str(), buf, nSize);
        if (!bRet)
        {
            printf("Error: read bin failed!\n");
            free(buf);
            return true;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_WRITE_REF, buf, nSize);
        if (rc != openni::STATUS_OK)
        {
            printf("%s,%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
            free(buf);
        }
        else
        {
            printf("Write reference success\n");
            free(buf);
        }
    }

    cout << "Usage: " << Command[0] << " < Set Params> ..." << endl;

    return true;
}

bool SetSoftwareAlignParams(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 2)
    {
        //write bin
        std::string swAlignFile = Command[1].c_str();
        if (swAlignFile.empty() || swAlignFile.find_last_of("bin") <= 0)
        {
            printf("Error: Please drag in a bin file!\n");
            return true;
        }

        uint8_t *buf = NULL;
        uint32_t nSize;
        nSize = getFileSize((char *)swAlignFile.c_str());
        buf = (unsigned char *)malloc(sizeof(unsigned char)*nSize);
        bool bRet = readFile((char *)swAlignFile.c_str(), buf, nSize);
        if (!bRet)
        {
            printf("Error: read bin failed!\n");
            free(buf);
            return true;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_WRITE_SW_ALIGN_PARAM, buf, nSize);
        if (rc != openni::STATUS_OK)
        {
            printf("%s,%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
            free(buf);
        }
        else
        {
            printf("Write software alignment parameters success\n");
            free(buf);
        }
    }

    cout << "Usage: " << Command[0] << " < Set Params> ..." << endl;

    return true;
}

bool SetHardwareAlignParams(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 3)
    {
        int nRegNum;
        if (!atoi2(Command[1].c_str(), &nRegNum))
        {
            printf("Id (%s) isn't a number\n", Command[1].c_str());
            return true;
        }

        //write bin
        std::string hwAlignFile = Command[2].c_str();
        if (hwAlignFile.empty() || hwAlignFile.find_last_of("bin") <= 0)
        {
            printf("Error: Please drag in a bin file!\n");
            return true;
        }

        OBCfgHwD2cDistortion hwD2cDistortion;
        uint32_t nSize = getFileSize((char *)hwAlignFile.c_str());
        if (nSize > sizeof(hwD2cDistortion.data))
        {
            printf("Error: bin file is too large!\n");
            return true;
        }

        memset(hwD2cDistortion.data, 0, sizeof(hwD2cDistortion.data));
        bool bRet = readFile((char*)hwAlignFile.c_str(), hwD2cDistortion.data, nSize);
        if (!bRet)
        {
            printf("Error: read bin failed!\n");
            return true;
        }

        hwD2cDistortion.nSize = nSize;
        hwD2cDistortion.nRegNum = nRegNum;

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_WRITE_HW_ALIGN_PARAM, hwD2cDistortion);
        if (rc != openni::STATUS_OK)
        {
            printf("%s,%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Write hardware alignment parameters success\n");
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " < Set Params> nRegNum d2cPath..." << endl;

    return true;
}

bool SetHardwareDistParams(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 2)
    {
        //write bin
        std::string hwDistFile = Command[1].c_str();
        if (hwDistFile.empty() || hwDistFile.find_last_of("bin") <= 0)
        {
            printf("Error: Please drag in a bin file!\n");
            return true;
        }

        uint8_t *buf = NULL;
        uint32_t nSize;
        nSize = getFileSize((char *)hwDistFile.c_str());
        buf = (unsigned char *)malloc(sizeof(unsigned char)*nSize);
        bool bRet = readFile((char *)hwDistFile.c_str(), buf, nSize);
        if (!bRet)
        {
            printf("Error: read bin failed!\n");
            free(buf);
            return true;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_WRITE_HW_DISTORTION_PARAM, buf, nSize);
        if (rc != openni::STATUS_OK)
        {
            printf("%s,%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
            free(buf);
        }
        else
        {
            printf("Write hardware distortion parameters success\n");
            free(buf);
        }
    }

    cout << "Usage: " << Command[0] << " < Set Params> ..." << endl;

    return true;
}

bool GetIRSensorModel(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 irModel = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_IRS_MODEl, &irModel);
        if (rc != XN_STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("IR model: 0x%x\n", irModel);

        }

    }

    return true;
}

bool GetRgbSensorModel(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 rgbModel = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_RGBS_MODEl, &rgbModel);
        if (rc == openni::STATUS_NOT_SUPPORTED)
        {
            printf("Error: not support for device acquisition model.\n");
            return true;
        }
        else if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("RGB model: 0x%x\n", rgbModel);
        }

    }

    return true;
}

bool I2CReadCameraParam(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        OBCameraParams cameraParam;
        //int dataSize = sizeof(cameraParam);
        memset(&cameraParam, 0, sizeof(cameraParam));

        XnParamFlashData readFlash;
        readFlash.pData = (uint8_t *)&cameraParam;
        readFlash.nOffset = 0x70000;
        readFlash.nSize = sizeof(cameraParam);

        int dataSize = sizeof(XnParamFlashData) + readFlash.nSize - sizeof(readFlash.pData);

        memset(&cameraParam, 0, sizeof(cameraParam));
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_I2C_READ_FLASH_MIPI, (uint8_t *)&readFlash, &dataSize);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            for (int i = 0; i < 4; i++)
            {
                printf("cameraParam.l_intr_p[%d] =%lf\n", i, cameraParam.l_intr_p[i]);
            }

            for (int i = 0; i < 4; i++)
            {
                printf("cameraParam.r_intr_p[%d] =%lf\n", i, cameraParam.r_intr_p[i]);
            }


            for (int i = 0; i < 9; i++)
            {
                printf("cameraParam.r2l_r[%d] =%lf\n", i, cameraParam.r2l_r[i]);
            }

            for (int i = 0; i < 3; i++)
            {
                printf("cameraParam.r2l_t[%d] =%lf\n", i, cameraParam.r2l_t[i]);
            }

            for (int i = 0; i < 5; i++)
            {
                printf("cameraParam.l_k[%d] =%lf\n", i, cameraParam.l_k[i]);
            }

            for (int i = 0; i < 5; i++)
            {
                printf("cameraParam.r_k[%d] =%lf\n", i, cameraParam.r_k[i]);
            }
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <set ObCameraParam> ..." << endl;
    return true;
}

bool GetZ0_Baseline(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        OBZ0Baseline z0Baseline = { 0 };
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_Z0_BASELINE, &z0Baseline);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Z0 =%.4f\n", z0Baseline.fZ0);
            printf("baseline =%.4f\n", z0Baseline.fBaseline);
            printf("\n");
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " < get z0_baseline number> ..." << endl;
    return true;
}

//
bool SetZ0_Baseline(openni::Device& Device, vector<string>& Command)
{
    OBZ0Baseline cfgZ0_Baseline;
    memset(&cfgZ0_Baseline, 0, sizeof(OBZ0Baseline));

    if (Command.size() == 3 || Command.size() == 2)
    {
        if (Command.size() == 2 && Command[1] != "clear")
        {
            cout << "Usage: " << Command[0] << " <setz0_baseline> clear" << endl;
            return true;
        }

        if (Command.size() == 3)
        {
            float fZ0 = (float)atof(Command[1].c_str());
            float fBaseline = (float)atof(Command[2].c_str());

            cfgZ0_Baseline.fZ0 = fZ0;
            cfgZ0_Baseline.fBaseline = fBaseline;
        }
        else if (Command[1] == "clear")
        {
            memset(&cfgZ0_Baseline, 0xFF, sizeof(OBZ0Baseline));
        }
        else
        {
            cout << "Usage: " << Command[0] << " < setz0_baseline>  z0 baseline  || <setz0_baseline> clear" << endl;
            return true;
        }

        //
        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_Z0_BASELINE, cfgZ0_Baseline);
        if (rc != openni::STATUS_OK)
        {
            printf("%s,%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
            return true;
        }
        else
        {
            printf("set z0 and baseline success\n");
        }
    }
    else
    {
        cout << "Usage: " << Command[0] << " < setz0_baseline>  z0 baseline  || <setz0_baseline> clear" << endl;
    }

    return true;
}


bool doAeParamsStruct(openni::Device& Device, vector<string>& Command, AeParamsStruct& aeStruct, bool bRead = true)
{
    string strValue = Command[1];
    int nPropertyID = -1;
    if (isStringEqual(strValue, "flood"))
    {
        nPropertyID = XN_MODULE_PROPERTY_FLOOD_AE_OPTIONS;
    }
    else if (isStringEqual(strValue, "emitter"))
    {
        nPropertyID = XN_MODULE_PROPERTY_EMITTER_AE_OPTIONS;
    }
    else
    {
        printf("Error:no %s parameter\n", strValue.c_str());
        return false;
    }
    openni::Status rc = bRead ? Device.getProperty(nPropertyID, &aeStruct) : Device.setProperty(nPropertyID, aeStruct);
    if (rc != openni::STATUS_OK)
    {
        printf("Error: %s\n", openni::OpenNI::getExtendedError());
        return false;
    }
    return true;
}

void initAeOptionsName()
{
    if (g_vecAEOptionsName.empty())
    {
        g_vecAEOptionsName.push_back("expTime");
        g_vecAEOptionsName.push_back("AGain");
        g_vecAEOptionsName.push_back("laserCurrent");
        g_vecAEOptionsName.push_back("targetBrightness");
        g_vecAEOptionsName.push_back("targetThd");
        g_vecAEOptionsName.push_back("centerWeight");
        g_vecAEOptionsName.push_back("skipFrame");
        g_vecAEOptionsName.push_back("smoothSteps");
        g_vecAEOptionsName.push_back("dealy_ms");
        g_vecAEOptionsName.push_back("meterMethod");
        g_vecAEOptionsName.push_back("expTimeAdj");
        g_vecAEOptionsName.push_back("AGainAdj");
        g_vecAEOptionsName.push_back("laserCurrentAdj");
    }
}

void printfAEOptionInfo(AeParamsStruct& aeValue, int i)
{
    if (i == EXPTIME)
    {
        printf("expTime:%d~%d\n", aeValue.expTime.min, aeValue.expTime.max);
    }
    else if (i == AGAIN)
    {
        printf("AGain:%d~%d\n", aeValue.AGain.min, aeValue.AGain.max);
    }
    else if (i == LASERCURRENT)
    {
        printf("laserCurrent:%d~%d\n", aeValue.laserCurrent.min, aeValue.laserCurrent.max);
    }
    else if (i == TARGETBRIGHTNESS)
    {
        printf("targetBrightness:%d\n", aeValue.targetBrightness);
    }
    else if (i == TARGETTHD)
    {
        printf("targetThd:%d\n", aeValue.targetThd);
    }
    else if (i == METERMETHOD)
    {
        int meterType = aeValue.meterMethod;
        string strMeterValue = "";
        switch (meterType)
        {
        case CENTER_METERING:
            strMeterValue = "CENTER_METERING";
            break;
        case AVERRAGE_METERING:
            strMeterValue = "AVERRAGE_METERING";
            break;
        case SPOT_METERING:
            strMeterValue = "SPOT_METERING";
            break;
        case ROI_METERING:
            strMeterValue = "ROI_METERING";
            break;
        case AUTO_METERING:
            strMeterValue = "AUTO_METERING";
            break;
        case WEIGHT_METERING:
            strMeterValue = "WEIGHT_METERING";
            break;
        case INV_TRIANGLE:
            strMeterValue = "INV_TRIANGLE";
            break;
        default:
            break;
        }
        printf("meterMethod:%s\n", strMeterValue.c_str());
    }
    else if (i == SKIPFRAME)
    {
        printf("skipFrame:%d\n", aeValue.skipFrame);
    }
    else if (i == SMOOTHSTEPS)
    {
        printf("smoothSteps:%d\n", aeValue.smoothSteps);
    }
    else if (i == EXPTIMEADJ)
    {
        printf("expTimeAdj:%s\n", aeValue.expTimeAdj ? "true" : "false");
    }
    else if (i == AGAINADJ)
    {
        printf("AGainAdj:%s\n", aeValue.AGainAdj ? "true" : "false");
    }
    else if (i == LASERCURRENTADJ)
    {
        printf("laserCurrentAdj:%s\n", aeValue.laserCurrentAdj ? "true" : "false");
    }
}

bool GetAEOptions(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 3)
    {
        initAeOptionsName();
        AeParamsStruct aeValue;
        if (!doAeParamsStruct(Device, Command, aeValue))
        {
            return true;
        }
        string strValue = Command[2];
        for (int i = 0; i < (int)g_vecAEOptionsName.size(); ++i)
        {
            if (isStringEqual(strValue, "all") || isStringEqual(g_vecAEOptionsName[i], strValue))
            {
                printfAEOptionInfo(aeValue, i);
                if (isStringEqual(strValue, "all"))
                {
                    continue;
                }
                break;
            }
        }
        return true;
    }
    cout << "Usage: " << Command[0] << " flood/emitter  parameter" << endl;
    return true;
}

bool SetAEOptions(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() >= 4)
    {
        initAeOptionsName();
        AeParamsStruct aeValue;
        if (!doAeParamsStruct(Device, Command, aeValue))
        {
            return true;
        }

        string strValue = Command[2];
        for (int i = 0; i < (int)g_vecAEOptionsName.size(); ++i)
        {
            if (isStringEqual(g_vecAEOptionsName[i], strValue))
            {
                if (i == EXPTIME || i == AGAIN || i == LASERCURRENT)
                {
                    if (Command.size() == 4)
                    {
                        printf("Error Parames\n");
                        return true;
                    }
                    XnUInt32 nMin, nMax;
                    if (!atoi2(Command[3].c_str(), (int*)&nMin) || \
                        !atoi2(Command[4].c_str(), (int*)&nMax))
                    {
                        printf("Error Parames\n");
                        return true;
                    }
                    switch (i)
                    {
                    case EXPTIME:
                        aeValue.expTime.min = nMin;
                        aeValue.expTime.max = nMax;
                        break;
                    case AGAIN:
                        aeValue.AGain.min = nMin;
                        aeValue.AGain.max = nMax;
                        break;
                    default:
                        aeValue.laserCurrent.min = nMin;
                        aeValue.laserCurrent.max = nMax;
                        break;
                    }

                }
                else if (i == TARGETBRIGHTNESS || i == TARGETTHD || i == SKIPFRAME || i == SMOOTHSTEPS)
                {
                    XnUInt32 nValue;
                    if (!atoi2(Command[3].c_str(), (int*)&nValue))
                    {
                        printf("Error Parames\n");
                        return true;
                    }

                    switch (i)
                    {
                    case TARGETBRIGHTNESS:
                        aeValue.targetBrightness = nValue;
                        break;
                    case TARGETTHD:
                        aeValue.targetThd = nValue;
                        break;
                    case SKIPFRAME:
                        aeValue.skipFrame = nValue;
                        break;
                    default:
                        aeValue.smoothSteps = nValue;
                        break;
                    }

                }
                else if (i == METERMETHOD)
                {
                    string strAeValue = Command[3];
                    METER_METHOD meterValue;
                    if (isStringEqual(strAeValue, "CENTER_METERING"))
                    {
                        meterValue = CENTER_METERING;
                    }
                    else if (isStringEqual(strAeValue, "AVERRAGE_METERING"))
                    {
                        meterValue = AVERRAGE_METERING;
                    }
                    else if (isStringEqual(strAeValue, "SPOT_METERING"))
                    {
                        meterValue = SPOT_METERING;
                    }
                    else if (isStringEqual(strAeValue, "ROI_METERING"))
                    {
                        meterValue = ROI_METERING;
                    }
                    else if (isStringEqual(strAeValue, "AUTO_METERING"))
                    {
                        meterValue = AUTO_METERING;
                    }
                    else if (isStringEqual(strAeValue, "WEIGHT_METERING"))
                    {
                        meterValue = WEIGHT_METERING;
                    }
                    else if (isStringEqual(strAeValue, "INV_TRIANGLE"))
                    {
                        meterValue = INV_TRIANGLE;
                    }
                    else
                    {
                        XnUInt32 nValue;
                        if (!atoi2(Command[3].c_str(), (int*)&nValue))
                        {
                            printf("Error Parames\n");
                            return true;
                        }
                        meterValue = (METER_METHOD)nValue;
                    }
                    aeValue.meterMethod = meterValue;
                }
                else if (i == EXPTIMEADJ || i == AGAINADJ || i == LASERCURRENTADJ)
                {
                    string strAeValue = Command[3];
                    bool bADJ;
                    if (isStringEqual(strAeValue, "true"))
                    {
                        bADJ = true;
                    }
                    else if (isStringEqual(strAeValue, "false"))
                    {
                        bADJ = false;
                    }
                    else
                    {
                        XnUInt32 nValue;
                        if (!atoi2(Command[3].c_str(), (int*)&nValue))
                        {
                            printf("Error Parames\n");
                            return true;
                        }
                        if (nValue == 0)
                        {
                            bADJ = false;
                        }
                        else if (nValue == 1)
                        {
                            bADJ = true;
                        }
                        else
                        {
                            printf("Error Parames\n");
                            return true;
                        }
                    }
                    switch (i)
                    {
                    case EXPTIMEADJ:
                        aeValue.expTimeAdj = bADJ;
                        break;
                    case AGAINADJ:
                        aeValue.AGainAdj = bADJ;
                        break;
                    default:
                        aeValue.laserCurrentAdj = bADJ;
                        break;
                    }

                }
                if (!doAeParamsStruct(Device, Command, aeValue, false))
                {
                    return true;
                }
                break;
            }
        }
        return true;
    }
    cout << "Usage: " << Command[0] << " flood/emitter  parameter  value" << endl;
    return true;
}

bool WriteDistortionFlash(char * filename, openni::Device& Device)
{
    if (filename == NULL)
    {
        return false;
    }

    uint32_t nSize = getFileSize(filename);
    printf("read data size %d\n", nSize);
    uint32_t nMemorySize = 0;
    if (nSize % 2 != 0)
    {
        nMemorySize = nSize + 1;
    }
    else
    {
        nMemorySize = nSize;
    }

    XnDistortionParam distortionParam;
    distortionParam.data = (unsigned char *)malloc(sizeof(unsigned char)*nMemorySize);
    memset(distortionParam.data, 0, sizeof(unsigned char)*nMemorySize);
    bool bRet = readFile(filename, distortionParam.data, nSize);
    if (!bRet)
    {
        free(distortionParam.data);
        return false;
    }

    distortionParam.nSize = nSize;
    openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_DISTORTION_PARAM, distortionParam);
    if (rc != openni::STATUS_OK)
    {
        printf("%s,%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
        free(distortionParam.data);
        return false;
    }
    else
    {
        printf("Write distortion param flash success\n");

        //read flash to verify
        XnDistortionParam readFlash;
        readFlash.data = (unsigned char *)malloc(sizeof(unsigned char)*nSize);
        readFlash.nSize = nSize;

        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_DISTORTION_PARAM, &readFlash);
        if (rc != openni::STATUS_OK)
        {
            printf("%s,%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
            free(distortionParam.data);
            free(readFlash.data);
            return false;
        }
        else
        {
            //verify data
            bool status = false;
            for (uint32_t i = 0; i < nSize; i++)
            {
                if (distortionParam.data[i] != readFlash.data[i])
                {
                    status = true;
                    break;
                }
            }

            free(distortionParam.data);
            free(readFlash.data);

            if (status)
            {
                printf("read distortion param flash verify failed\n");
                return false;
            }
            else
            {
                printf("read distortion param flash verify success\n");
                return true;
            }
        }

    }

}

bool ReadDistortionParamFlash(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        //read flash to verify
        int nSize = 512 * 1024;
        XnDistortionParam readFlash;
        readFlash.data = (unsigned char *)malloc(sizeof(unsigned char)*nSize);
        readFlash.nSize = nSize;

        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_DISTORTION_PARAM, &readFlash);
        if (rc != openni::STATUS_OK)
        {
            printf("%s,%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
            free(readFlash.data);
            return false;
        }
        else
        {
            //verify data
            printf("read distortion param flash verify success dataSize = %d \n", readFlash.nSize);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <updateUsbAppCode> ..." << endl;
    return true;
}

bool WriteDistortionParamFlash(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 2)
    {
        std::string strBin = Command[1].c_str();
        WriteDistortionFlash((char *)strBin.c_str(), Device);
        return true;
    }
    cout << "Usage: " << Command[0] << " <updateUsbAppCode> ..." << endl;
    return true;
}

bool DistortionCalEnable(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 2)
    {

        int nStatus;
        if (!atoi2(Command[1].c_str(), &nStatus))
        {
            printf("Can't understand '%s' \n", Command[1].c_str());
            return true;
        }

        switch (nStatus)
        {
        case 0:
        case 1:
        case 2:
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_DISTORTION_STATE, (XnUInt32)nStatus);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
        }

        break;
        default:
            printf("param value range is 0-2 \n");
            break;

        }
        return true;
    }

    cout << "Usage: " << Command[0] << " < param value ��0-2��> ..." << endl;
    return true;
}

bool GetDistortionCalStatus(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_DISTORTION_STATE, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Distortion calibration status: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " < get distortion calibration status> ..." << endl;
    return true;
}

bool GetPdEnableStatus(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 pdEnable = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_PD_ENABLE_STATUS, &pdEnable);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Pd enable status: %d\n", pdEnable);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " ..." << endl;
    return true;
}

bool SetPdEnableStatus(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        XnBool pdEnable = FALSE;
        if (Command[1] == "on")
        {
            pdEnable = TRUE;
        }
        else if (Command[1] == "off")
        {
            pdEnable = FALSE;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_PD_ENABLE_STATUS, (XnUInt32)pdEnable);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Pd status switch success!\n");
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " ..." << endl;
    return true;
}

bool GetPdAlertStatus(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 pdAlert = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_PD_ALERT_STATUS, &pdAlert);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Pd alert status: %d\n", pdAlert);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " ..." << endl;
    return true;
}

bool GetPdUpperThreshold(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 pdUpperValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_PD_UPPER_TLV, &pdUpperValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Pd upper threshold: %d\n", pdUpperValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " ..." << endl;
    return true;
}

bool SetPdUpperThreshold(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        int nValue;
        if (!atoi2(Command[1].c_str(), &nValue))
        {
            printf("Can't understand '%s' as threshold", Command[1].c_str());
            return true;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_PD_UPPER_TLV, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Set pd upper threshold success!\n");
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " ..." << endl;
    return true;
}

bool GetPdLowerThreshold(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 pdUpperValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_PD_LOWER_TLV, &pdUpperValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Pd lower threshold: %d\n", pdUpperValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " ..." << endl;
    return true;
}

bool SetPdLowerThreshold(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        int nValue;
        if (!atoi2(Command[1].c_str(), &nValue))
        {
            printf("Can't understand '%s' as threshold", Command[1].c_str());
            return true;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_PD_LOWER_TLV, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("set pd lower threshold success!\n");
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " ..." << endl;
    return true;
}

bool GetPd_CurrentThreshold(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        OBPdThreshold obPdThreshold;
        memset(&obPdThreshold, 0, sizeof(OBPdThreshold));
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_PD_CUR_TLV, &obPdThreshold);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Pd current threshold: %d\n", obPdThreshold.PdThreshold);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " ..." << endl;
    return true;
}

bool GetBootLoaderPts(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 bootLoaderPts = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_BOOTLOADER_PTS, &bootLoaderPts);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Bootloader protection status: %d\n", bootLoaderPts);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " ..." << endl;
    return true;
}

bool SetBootLoaderPts(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        XnBool bootLoaderPts = FALSE;
        if (Command[1] == "on")
        {
            bootLoaderPts = TRUE;
        }
        else if (Command[1] == "off")
        {
            bootLoaderPts = FALSE;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_BOOTLOADER_PTS, (XnUInt32)bootLoaderPts);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Set bootloader protection status success!\n");
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " ..." << endl;
    return true;
}

bool GetMipi_CoreBroadId(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_CORE_BROAD_FLASH_ID, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("core broad id: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " ..." << endl;
    return true;
}

bool SetDepthConfig(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 2)
    {
        std::string hwDistFile = Command[1].c_str();
        if (hwDistFile.empty() || hwDistFile.find_last_of("bin") <= 0)
        {
            printf("Error: Please drag in a bin file!\n");
            return true;
        }

        uint8_t *buf = NULL;
        uint32_t nSize;
        nSize = getFileSize((char *)hwDistFile.c_str());
        buf = (unsigned char *)malloc(sizeof(unsigned char)*nSize);
        bool bRet = readFile((char *)hwDistFile.c_str(), buf, nSize);
        if (!bRet)
        {
            printf("Error: read bin failed!\n");
            free(buf);
            return true;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_WRITE_DEPTH_CONFIG, buf, nSize);
        if (rc != openni::STATUS_OK)
        {
            printf("%s,%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
            free(buf);
        }
        else
        {
            printf("Write depth config success\n");
            free(buf);
        }
    }

    cout << "Usage: " << Command[0] << " < Set Params> ..." << endl;

    return true;
}

bool LaserTimeSet(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {

        int nValue;
        if (!atoi2(Command[1].c_str(), &nValue))
        {
            printf("Can't understand '%s' as laser time", Command[1].c_str());
            return true;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_LASER_TIME, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <Laser time value> ..." << endl;
    return true;
}

//get laser time
bool GetLaserTime(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_LASER_TIME, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("laserTime: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <Laser Time Value> ..." << endl;
    return true;
}

bool PostFilterThresholdSet(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {

        int nValue;
        if (!atoi2(Command[1].c_str(), &nValue))
        {
            printf("Can't understand '%s' as post filter threshold", Command[1].c_str());
            return true;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_POSTFILTER_THRESHOLD, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <post filter threshold value> ..." << endl;
    return true;
}

bool GetPostFilterThreshold(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_POSTFILTER_THRESHOLD, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("post filter threshold: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <post filter threshold Value> ..." << endl;
    return true;
}

bool GetZpps(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnDouble nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_ZPPS, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("zpps: %f\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <zpps Value> ..." << endl;
    return true;
}

bool IrGainDefSet(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {

        int nGainValue;
        if (!atoi2(Command[1].c_str(), &nGainValue))
        {
            printf("Can't understand '%s' as gain", Command[1].c_str());
            return true;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_IRGAIN_FLASH, (XnUInt32)nGainValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <IrGain gainValue> ..." << endl;
    return true;
}

bool IrExpDefSet(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {

        int nValue;
        if (!atoi2(Command[1].c_str(), &nValue))
        {
            printf("Can't understand '%s' as exp", Command[1].c_str());
            return true;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_IREXP_FLASH, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <Irexp expValue> ..." << endl;
    return true;
}

bool GetIrGainDef(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nGainValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_IRGAIN_FLASH, &nGainValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("irGain Def: %d\n", nGainValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << "..." << endl;
    return true;
}

//get IrExp def
bool GetIrExpDef(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_IREXP_FLASH, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("irExp Def: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << "..." << endl;
    return true;
}

bool PostFilterThresholdDefSet(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {

        int nValue;
        if (!atoi2(Command[1].c_str(), &nValue))
        {
            printf("Can't understand '%s' as post filter threshold", Command[1].c_str());
            return true;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_POSTFILTER_THRESHOLD_FLASH, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <PostFilterThreshold  Def nValue> ..." << endl;
    return true;
}

bool LaserCurrentDefSet(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {

        int nValue;
        if (!atoi2(Command[1].c_str(), &nValue))
        {
            printf("Can't understand '%s' as laser current", Command[1].c_str());
            return true;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_LASER_CURRENT_FLASH, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <Laser Current Def Value> ..." << endl;
    return true;
}

bool LaserTimeDefSet(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {

        int nValue;
        if (!atoi2(Command[1].c_str(), &nValue))
        {
            printf("Can't understand '%s' as laser time", Command[1].c_str());
            return true;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_LASER_TIME_FLASH, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <Laser time Def Value> ..." << endl;
    return true;
}

bool GetPostFilterThresholdDef(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_POSTFILTER_THRESHOLD_FLASH, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("PostFilter Def: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << "  ..." << endl;
    return true;
}

bool GetLaserCurrentDef(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_LASER_CURRENT_FLASH, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Laser Current Def: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << "  ..." << endl;
    return true;
}

bool GetLaserTimeDef(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_LASER_TIME_FLASH, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("Laser time Def: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << "..." << endl;
    return true;
}


bool SetTofSenEnable(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        if (Command[1] == "on")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_TOF_SENSOR_ENABLE, (XnUInt32)TRUE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
        else if (Command[1] == "off")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_TOF_SENSOR_ENABLE, (XnUInt32)FALSE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
    }

    cout << "Usage: " << Command[0] << " <TofEnable|get|on|off> ..." << endl;
    return true;
}

bool GetTofSenEnable(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 enableStatus = TRUE;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_TOF_SENSOR_ENABLE, &enableStatus);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("TOF sensor enable status: %d\n", enableStatus);
        }
        return true;
    }

    cout << "Usage: " << Command[0] << " <Tof sensor enable Value> ..." << endl;
    return true;
}


bool GetTofSenMeasure(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 measure = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_TOF_SENSOR_MEA_RESULT, &measure);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("TOF Measuring Distance : %d\n", measure);
        }
        return true;
    }

    cout << "Usage: " << Command[0] << " <TOF Measuring Distance Value> ..." << endl;
    return true;
}

bool GetTofSenAppId(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 appid = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_TOF_SENSOR_APP_ID, &appid);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("TOF App id : %d\n", appid);
        }
        return true;
    }

    cout << "Usage: " << Command[0] << " <TOF App id Value> ..." << endl;
    return true;
}

bool TofSenCalibration(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_TOF_SENSOR_CAL, (XnUInt32)TRUE);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        return true;
    }

    cout << "Usage: " << Command[0] << " <TOF Sensor calibration> ..." << endl;
    return true;
}

bool SetTofSenAppStart(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        if (Command[1] == "on")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_TOF_SENSOR_APP_START, (XnUInt32)TRUE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
        else if (Command[1] == "off")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_TOF_SENSOR_APP_START, (XnUInt32)FALSE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
    }

    cout << "Usage: " << Command[0] << " <TofEnable|get|on|off> ..." << endl;
    return true;
}

bool GetCupVerifyVersion(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        CupCertify cupVersion = { 0 };
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_CUP_VERIFY_VERSION, &cupVersion);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("product type:%s\n", cupVersion.ProductType);
            printf("fireware version:");
            printf("%c.", cupVersion.FirewareVersion[0]);
            printf("%c.", cupVersion.FirewareVersion[1]);
            printf("%c%c\n", cupVersion.FirewareVersion[2], cupVersion.FirewareVersion[3]);
            return true;
        }
    }
    cout << "Usage: " << Command[0] << " < GetCupVerifyVersion > ..." << endl;
    return true;
}

bool GetTofSenCalibrationParams(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        OBTofSensorCalParams tofCalParams;
        memset(&tofCalParams, 0, sizeof(OBTofSensorCalParams));
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_TOF_SENSOR_CAL_PARAMS, &tofCalParams);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
            return true;
        }

        int pLen = sizeof(tofCalParams.calParams);

        printf("Get TOF Sensor calibration params success: !\n");
        for (int i = 0; i < pLen; i++)
        {
            printf("0x%02x\n", tofCalParams.calParams[i]);
        }
    }

    cout << "Usage: " << Command[0] << " <Get TOF Sensor calibration params> ..." << endl;
    return true;
}


bool SetTofSenCalibrationParams(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 15)
    {
        OBTofSensorCalParams tofCalParams;
        memset(&tofCalParams, 0, sizeof(OBTofSensorCalParams));

        int source[14] = { 0 };
        for (int j = 0; j < 14; j++)
        {
            atoi2(Command[j + 1].c_str(), &source[j]);
            tofCalParams.calParams[j] = (uint8_t)source[j];
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_TOF_SENSOR_CAL_PARAMS, tofCalParams);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        return true;
    }

    cout << "Usage: " << Command[0] << " <Set TOF Sensor calibration params> ..." << endl;
    return true;
}

//
bool DepthIrModeSet(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {

        int nValue;
        if (!atoi2(Command[1].c_str(), &nValue))
        {
            printf("Can't understand '%s' as depthIrMode", Command[1].c_str());
            return true;
        }

        if (nValue > 2 || nValue < 0)
        {
            printf("input value out of range\n");
            return true;
        }

        openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_DEPTH_IR_MODE, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            //printf("Error: %s\n", openni::OpenNI::getExtendedError());
            printf("depth ir mode set faild,maybe device unsupport\n");
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <DepthIrMode Value> ..." << endl;
    return true;
}


bool GetDepthIrMode(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() == 1)
    {
        XnUInt32 nValue = 0;
        openni::Status rc = Device.getProperty(XN_MODULE_PROPERTY_DEPTH_IR_MODE, &nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("depthIrMode: %d\n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <DepthIrMode Value> ..." << endl;
    return true;
}

bool TecEnable(openni::Device& Device, vector<string>& Command)
{
    if (Command.size() > 1)
    {
        if (Command[1] == "on")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_TEC_ENABLE, (XnUInt32)TRUE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
        else if (Command[1] == "off")
        {
            openni::Status rc = Device.setProperty(XN_MODULE_PROPERTY_TEC_ENABLE, (XnUInt32)FALSE);
            if (rc != openni::STATUS_OK)
            {
                printf("Error: %s\n", openni::OpenNI::getExtendedError());
            }
            return true;
        }
    }

    cout << "Usage: " << Command[0] << " <on|off> ..." << endl;
    return true;
}

bool setSubtractBackgroudMode(openni::Device& device, vector<string>& Command)
{
    if (Command.size() < 2){
        printf(" set subtract backgroud failed !!\n");
        return false;
    }
    int nValue;
    if (!atoi2(Command[1].c_str(), &nValue))
    {
        printf("Can't understand '%s' as depthIrMode", Command[1].c_str());
        return true;
    }

    if (nValue > 2 || nValue < 0)
    {
        printf("input value out of range\n");
        return true;
    }

    printf("set subtract backgroud mode: %d\n", nValue);
    device.setProperty(ONI_DEVICE_PROPERTY_ENABLE_SUBTRACT_BG, nValue);
    return true;
}

bool getSubtractBackgroudMode(openni::Device& device, vector<string>& Command)
{
    //printf("get subtract Command.size: %d\n", Command.size());
    if (Command.size() < 0){
        return false;
    }
    int enable = 0;
    device.getProperty(ONI_DEVICE_PROPERTY_STATE_SUBTRACT_BG, &enable);
    printf("get subtract backgroud mode: %d\n", enable);
    return true;
}

bool LoadFileImpl(openni::Device& device, const std::string& filePath, const openni::FileSuffix suffix, const openni::FileCategory category)
{
    if (filePath.empty())
    {
        printf("File path can not be empty...\n");
        return true;
    }

    printf("Loading file <%s>...\n", filePath.c_str());
    openni::Status rc = device.getDeviceSettings()->loadFile(filePath.c_str(), suffix, category);
    if (openni::STATUS_OK != rc)
    {
        printf("Failed to load file <%s>, error <%s>\n", filePath.c_str(), openni::OpenNI::getExtendedError());
        return true;
    }

    printf("File <%s> loaded successfully.\n", filePath.c_str());
    return true;
}

bool LoadFirmwareImage(openni::Device& device, vector<string>& command)
{
    if (command.size() < 2)
    {
        printf("Bad command..., Usage: LoadFirmwareImage <file>\n");
        return true;
    }

    return LoadFileImpl(device, command[1], openni::FILE_SUFFIX_BINARY, openni::FILE_CATEGORY_FW);
}

bool LoadD2CBin(openni::Device& device, vector<string>& command)
{
    if (command.size() < 2)
    {
        printf("Bad command..., Usage: LoadD2CBin <file>\n");
        return true;
    }

    return LoadFileImpl(device, command[1], openni::FILE_SUFFIX_BINARY, openni::FILE_CATEGORY_D2C);
}

bool LoadDepthBin(openni::Device& device, vector<string>& command)
{
    if (command.size() < 2)
    {
        printf("Bad command..., Usage: LoadDepthBin <file>\n");
        return true;
    }

    return LoadFileImpl(device, command[1], openni::FILE_SUFFIX_BINARY, openni::FILE_CATEGORY_CALIB);
}

bool LoadFilterBin(openni::Device& device, vector<string>& command)
{
    if (command.size() < 2)
    {
        printf("Bad command..., Usage: LoadFilterBin <file>\n");
        return true;
    }

    return LoadFileImpl(device, command[1], openni::FILE_SUFFIX_BINARY, openni::FILE_CATEGORY_FILTER);
}

bool LoadRomImage(openni::Device& device, vector<string>& command)
{
    if (command.size() < 2)
    {
        printf("Bad command..., Usage: LoadRomImage <file>\n");
        return true;
    }

    return LoadFileImpl(device, command[1], openni::FILE_SUFFIX_BINARY, openni::FILE_CATEGORY_ROM);
}

bool LoadCommonFile(openni::Device& device, vector<string>& command)
{
    if (command.size() < 2)
    {
        printf("Bad command..., Usage: LoadCommonFile <file>\n");
        return true;
    }

    return LoadFileImpl(device, command[1], openni::FILE_SUFFIX_NONE, openni::FILE_CATEGORY_COMMON);
}

bool GetSensorID(openni::Device& device, vector<string>& command)
{
    if (command.size() < 2)
    {
        printf("Bad command..., Usage: GetSensorID <1 IR|2 Color|3 Depth|4 Phase>\n");
        return true;
    }

    std::string strArg = command.at(1);
    int32_t type = std::atoi(strArg.c_str());
    if (type < openni::SENSOR_IR || type > openni::SENSOR_PHASE)
    {
        printf("Bad argument..., Usage: GetSensorID <1 IR|2 Color|3 Depth|4 Phase>\n");
        return true;
    }

    const openni::SensorID& id = device.getDeviceSettings()->getSensorID((openni::SensorType)type);
    if (openni::SENSOR_ID_NONE == id)
    {
        printf("Unknown sensor ID: sensor type (%d), error (%s)...\n", type, openni::OpenNI::getExtendedError());
        return true;
    }

    printf("Get sensor ID: <%d, 0x%x>\n", type, id);
    return true;
}

bool GetPlatformVersion(openni::Device& device, vector<string>& command)
{
    if (command.size() < 1)
    {
        printf("Bad command..., Usage: GetPlatformVersion\n");
        return true;
    }

    const char* pVersion = device.getDeviceSettings()->getPlatformVersion();
    if (strlen(pVersion) <= 0)
    {
        printf("Failed to get platform version: error <%s>\n", openni::OpenNI::getExtendedError());
        return true;
    }

    printf("Get platform version: <%s>\n", pVersion);
    return true;
}

bool GetGeneralSerialNumber(openni::Device& device, vector<string>& command)
{
    if (command.size() < 2)
    {
        printf("Bad command..., Usage: GetGeneralSerialNumber <1 IR|2 Color|3 Depth|4 Device|5 Product>\n");
        return true;
    }

    std::string strArg = command.at(1);
    int32_t type = std::atoi(strArg.c_str());
    if (type <= openni::SERIAL_NONE || type > openni::SERIAL_PRODUCT)
    {
        printf("Bad argument..., Usage: GetGeneralSerialNumber <1 IR|2 Color|3 Depth|4 Device|5 Product>\n");
        return true;
    }

    const char* pSerial = device.getDeviceSettings()->getSerialNumber((openni::SerialType)type);
    if (strlen(pSerial) <= 0)
    {
        printf("Failed to get serial number, type (%d), error (%s)...\n", type, openni::OpenNI::getExtendedError());
        return true;
    }

    char sn[64] = { 0 };
    for (uint32_t i = 0; i < strlen(pSerial); ++i)
    {
        sprintf(sn + i * 2, "%02x", pSerial[i]);
    }

    printf("Get general serial number: <%d, %s | %s>\n", type, pSerial, sn);
    return true;
}

bool SetGeneralSerialNumber(openni::Device& device, vector<string>& command)
{
    if (command.size() < 3)
    {
        printf("Bad command..., Usage: SetGeneralSerialNumber <1 IR|2 Color|3 Depth|4 Device|5 Product, serial number>\n");
        return true;
    }

    std::string strArg = command.at(1);
    int32_t type = std::atoi(strArg.c_str());
    if (type <= openni::SERIAL_NONE || type > openni::SERIAL_PRODUCT)
    {
        printf("Bad argument..., Usage: SetGeneralSerialNumber <1 IR|2 Color|3 Depth|4 Device|5 Product, serial number>\n");
        return true;
    }

    strArg = command.at(2);
    if (strArg.size() > ONI_MAX_STR)
    {
        printf("The length of general serial number is too long <%d>, it must be limited to <%d>...", (uint32_t)strArg.size(), ONI_MAX_STR);
        return true;
    }

    openni::Status rc = device.getDeviceSettings()->setSerialNumber((openni::SerialType)type, strArg.c_str());
    if (openni::STATUS_OK != rc)
    {
        printf("Error: %s\n", openni::OpenNI::getExtendedError());
        printf("Failed to set serial number: <%d, %s>, error <%s>...\n", type, strArg.c_str(), openni::OpenNI::getExtendedError());
        return true;
    }

    printf("Set serial number <%d, %s> successfully.\n", type, strArg.c_str());
    return true;
}

bool GetTOFWorkingMode(openni::Device& device, vector<string>& command)
{
    if (command.size() < 1)
    {
        printf("Bad command..., Usage: GetTOFWorkingMode\n");
        return true;
    }

    openni::FrequencyMode mode = device.getDeviceSettings()->getTOFWorkingMode();
    if (openni::FREQUENCY_MODE_NONE == mode)
    {
        printf("Failed to get TOF sensor working mode due to error(%s)...\n", openni::OpenNI::getExtendedError());
        return true;
    }

    std::string strMode;
    switch (mode)
    {
    case openni::SINGLE_FREQ_NOSHUFFLE:
        strMode = "single freq noshuffle";
        break;
    case openni::DUAL_FREQ_NOSHUFFLE:
        strMode = "dual freq noshuffle";
        break;
    case openni::SINGLE_FREQ_SHUFFLE:
        strMode = "single freq shuffle";
        break;
    case openni::DUAL_FREQ_SHUFFLE:
        strMode = "dual freq shuffle";
        break;
    case openni::SINGLE_FREQ_NOSHUFFLE_BINNING:
        strMode = "single freq noshuffle binning";
        break;
    case openni::DUAL_FREQ_NOSHUFFLE_BINNING:
        strMode = "dual freq noshuffle binning";
        break;
    case openni::SINGLE_FREQ_SHUFFLE_BINNING:
        strMode = "single freq shuffle binning";
        break;
    case openni::DUAL_FREQ_SHUFFLE_BINNING:
        strMode = "dual freq shuffle binning";
        break;
    default:
        strMode = "unknown freq mode";
    }

    printf("Get TOF sensor working mode: <%s>.\n", strMode.c_str());
    return true;
}

bool SetTOFWorkingMode(openni::Device& device, vector<string>& command)
{
    if (command.size() < 2)
    {
        printf("Bad command..., Usage: SetTOFWorkingMode <0 single freq nonshuffle|1 dual freq nonshuffle|2 single freq shuffle|3 dual freq shuffle|4 single freq nonshuffle binning|5 dual freq nonshuffle binning|6 single freq shuffle binning|7 dual freq shuffle binning>\n");
        return true;
    }

    std::string strArg = command.at(1);
    int32_t mode = std::atoi(strArg.c_str());
    if (mode <= openni::FREQUENCY_MODE_NONE || mode >= openni::FREQUENCY_MODE_MAX)
    {
        printf("Bad argument..., Usage: SetTOFWorkingMode <0 single freq nonshuffle|1 dual freq nonshuffle|2 single freq shuffle|3 dual freq shuffle|4 single freq nonshuffle binning|5 dual freq nonshuffle binning|6 single freq shuffle binning|7 dual freq shuffle binning>\n");
        return true;
    }

    openni::Status rc = device.getDeviceSettings()->setTOFWorkingMode((openni::FrequencyMode)mode);
    if (openni::STATUS_OK != rc)
    {
        printf("Failed to set TOF sensor working mode to <%s> due to error <%s>...\n", strArg.c_str(), openni::OpenNI::getExtendedError());
        return true;
    }

    printf("Set TOF sensor working mode to <%d> successfully.\n", mode);
    return true;
}

bool GetEmitterState(openni::Device& device, vector<string>& command)
{
    if (command.size() < 1)
    {
        printf("Bad command..., Usage: GetEmitterState\n");
        return true;
    }

    bool state = device.getDeviceSettings()->getEmitterState();
    printf("Get emitter state: <%s>.\n", state ? "ON" : "OFF");
    return true;
}

bool SetEmitterState(openni::Device& device, vector<string>& command)
{
    if (command.size() < 2)
    {
        printf("Bad command..., Usage: SetEmitterState <on|off>\n");
        return true;
    }

    bool state = false;
    std::string strArg = command.at(1);
    if (0 == strArg.compare("on"))
        state = true;
    else if (0 == strArg.compare("off"))
        state = false;
    else
    {
        printf("Usage: EmitterEnable <on|off>\n");
        return true;
    }

    openni::Status rc = device.getDeviceSettings()->setEmitterState(state);
    if (openni::STATUS_OK != rc)
    {
        printf("Failed to enable or disable emitter <%s> due to error <%s>...\n", strArg.c_str(), openni::OpenNI::getExtendedError());
        return true;
    }

    printf("Set emitter state <%s> successfully.\n", strArg.c_str());
    return true;
}

bool GetDeviceTimestamp(openni::Device& device, vector<string>& command)
{
    if (command.size() < 1)
    {
        printf("Bad command..., Usage: GetDeviceTimestamp\n");
        return true;
    }

    uint64_t timestamp = device.getDeviceSettings()->getTimestamp();
    printf("Get device timestamp: <%" PRIu64 "(ms)>.\n", timestamp);
    return true;
}

bool GetCalibration(openni::Device& device, vector<string>& command)
{
    if (command.size() < 1)
    {
        printf("Bad command..., Usage: GetCalibrationCamera\n");
        return true;
    }

    const openni::CameraMatrix &color = device.getDeviceSettings()->getCalibrationCamera().getCalibrationColor();
    printf("Color Calibration Data:\n");
    printf("width                 : %d\n", color.getResolutionX());
    printf("height                : %d\n", color.getResolutionY());
    printf("fx fy cx cy           : %-12.6f %-12.6f %-12.6f %-12.6f\n", color.getIntrinsics().fx, color.getIntrinsics().fy, color.getIntrinsics().cx, color.getIntrinsics().cy);
    printf("k1 k2 k3              : %-12.6f %-12.6f %-12.6f\n", color.getIntrinsics().k1, color.getIntrinsics().k2, color.getIntrinsics().k3);
    printf("p1 p2                 : %-12.6f %-12.6f\n", color.getIntrinsics().p1, color.getIntrinsics().p2);
    printf("translation           : %-12.6f %-12.6f %-12.6f\n", color.getExtrinsics().translation[0], color.getExtrinsics().translation[1], color.getExtrinsics().translation[2]);
    printf("rotation              : %-12.6f %-12.6f %-12.6f\n", color.getExtrinsics().rotation[0], color.getExtrinsics().rotation[1], color.getExtrinsics().rotation[2]);
    printf("                        %-12.6f %-12.6f %-12.6f\n", color.getExtrinsics().rotation[3], color.getExtrinsics().rotation[4], color.getExtrinsics().rotation[5]);
    printf("                        %-12.6f %-12.6f %-12.6f\n", color.getExtrinsics().rotation[6], color.getExtrinsics().rotation[7], color.getExtrinsics().rotation[8]);

    const openni::CameraMatrix &depth = device.getDeviceSettings()->getCalibrationCamera().getCalibrationDepth();
    printf("Depth Calibration Data:\n");
    printf("width                 : %d\n", depth.getResolutionX());
    printf("height                : %d\n", depth.getResolutionY());
    printf("fx fy cx cy           : %-12.6f %-12.6f %-12.6f %-12.6f\n", depth.getIntrinsics().fx, depth.getIntrinsics().fy, depth.getIntrinsics().cx, depth.getIntrinsics().cy);
    printf("k1 k2 k3              : %-12.6f %-12.6f %-12.6f\n", depth.getIntrinsics().k1, depth.getIntrinsics().k2, depth.getIntrinsics().k3);
    printf("p1 p2                 : %-12.6f %-12.6f\n", depth.getIntrinsics().p1, depth.getIntrinsics().p2);
    printf("translation           : %-12.6f %-12.6f %-12.6f\n", depth.getExtrinsics().translation[0], depth.getExtrinsics().translation[1], depth.getExtrinsics().translation[2]);
    printf("rotation              : %-12.6f %-12.6f %-12.6f\n", depth.getExtrinsics().rotation[0], depth.getExtrinsics().rotation[1], depth.getExtrinsics().rotation[2]);
    printf("                        %-12.6f %-12.6f %-12.6f\n", depth.getExtrinsics().rotation[3], depth.getExtrinsics().rotation[4], depth.getExtrinsics().rotation[5]);
    printf("                        %-12.6f %-12.6f %-12.6f\n", depth.getExtrinsics().rotation[6], depth.getExtrinsics().rotation[7], depth.getExtrinsics().rotation[8]);

    return true;
}

bool GetTemperCoeffRX(openni::Device& device, vector<string>& command)
{
    if (command.size() < 1)
    {
        printf("Bad command..., Usage: GetDeviceTimestamp\n");
        return true;
    }

    double coeff = device.getDeviceSettings()->getTemperatureCoefficientRX();
    printf("Get RX temperature coefficient: <%f>.\n", coeff);
    return true;
}

bool SetTemperCoeffRX(openni::Device& device, vector<string>& command)
{
    if (command.size() < 2)
    {
        printf("Bad command..., Usage: SetTemperCoeffRX <coefficient>\n");
        return true;
    }

    std::string strArg = command.at(1);
    double coeff = std::atof(strArg.c_str());
    openni::Status rc = device.getDeviceSettings()->setTemperatureCoefficientRX(coeff);
    if (openni::STATUS_OK != rc)
    {
        printf("Failed to set RX temperature coefficient <%f> due to error <%s>...\n", coeff, openni::OpenNI::getExtendedError());
        return true;
    }

    printf("Set RX temperature coefficient <%f> successfully.\n", coeff);
    return true;
}

bool StartService(openni::Device& device, vector<string>& command)
{
    if (command.size() < 3)
    {
        printf("Bad command..., Usage: StartService <adb|rndis on|off>\n");
        return true;
    }

    openni::ServiceType type = openni::SERVICE_MAX;
    std::string strType = command.at(1);
    if (0 == strType.compare("adb"))
        type = openni::SERVICE_ADB;
    else if (0 == strType.compare("rndis"))
        type = openni::SERVICE_RNDIS;
    else
    {
        printf("Usage: StartService <adb|rndis on|off>\n");
        return true;
    }

    openni::ServiceState state = openni::SERVICE_OFF;
    std::string strStatus = command.at(2);
    if (0 == strStatus.compare("on"))
        state = openni::SERVICE_ON;
    else if (0 == strStatus.compare("off"))
        state = openni::SERVICE_OFF;
    else
    {
        printf("Usage: StartService <adb|rndis on|off>\n");
        return true;
    }

    openni::Status rc = device.getDeviceSettings()->startService(type, state);
    if (openni::STATUS_OK != rc)
    {
        printf("Failed to start service <%s, %s>: error <%s>...\n", strType.c_str(), strStatus.c_str(), openni::OpenNI::getExtendedError());
        return true;
    }

    printf("Start service <%s, %s> successfully.\n", strType.c_str(), strStatus.c_str());
    return true;
}

bool ExecCmd(openni::Device& device, vector<string>& command)
{
    if (command.size() < 2)
    {
        printf("Bad command..., Usage: ExecCmd <cmd0, cmd1, cmd2...>\n");
        return true;
    }

    std::string strCmd;
    for (uint32_t i = 1; i < command.size(); ++i)
    {
        strCmd.append(command.at(i));
        if (i != command.size() - 1)
            strCmd.append(" ");
    }

    char* pResp = NULL;
    openni::Status ret = device.getDeviceSettings()->sendSerialCmd(strCmd.c_str(), &pResp);
    if (openni::STATUS_OK != ret)
    {
        printf("Failed to execute command: <%s>, error <%s>...\n", strCmd.c_str(), openni::OpenNI::getExtendedError());
        return true;
    }

    printf("Execute command <%s, %s> successfully.\n", strCmd.c_str(), pResp);
    return true;
}

bool SetMotorTest(openni::Device& device, vector<string>& Command)
{
    if (Command.size() > 1)
    {

        int nValue;
        if (!atoi2(Command[1].c_str(), &nValue))
        {
            printf("Can't understand '%s' as depthIrMode", Command[1].c_str());
            return true;
        }

        if (nValue != 1)
        {
            printf("input value out of range\n");
            return true;
        }

        openni::Status rc = device.setProperty(XN_MODULE_PROPERTY_MOTOR_TEST, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <SetMotorTest 1 > ..." << endl;
    return true;
}

bool GetMotorTestResult(openni::Device& device, vector<string>& command)
{
    if (command.size() < 1)
    {
        printf("Bad command...\n");
        return false;
    }

    XnUInt32 MotorStatus = 0;
    openni::Status rc = device.getProperty(XN_MODULE_PROPERTY_MOTOR_TEST, &MotorStatus);
    if (openni::STATUS_OK != rc)
    {
        printf("Failed  error (%s)\n", openni::OpenNI::getExtendedError());
        return false;
    }
    else
    {
        printf("MotorStatus: %d\n", MotorStatus);
    }

    cout << "Usage: " << command[0] << " <GetMotorTestResult> ..." << endl;
    return true;
}

bool SetMotorPosition(openni::Device& device, vector<string>& Command)
{
    if (Command.size() > 1)
    {

        int nValue;
        if (!atoi2(Command[1].c_str(), &nValue))
        {
            printf("Can't understand '%s' as depthIrMode", Command[1].c_str());
            return true;
        }

        if (nValue > 2 || nValue < 1)
        {
            printf("input value out of range\n");
            return true;
        }

        openni::Status rc = device.setProperty(XN_MODULE_PROPERTY_MOTOR_POSITION, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <SetMotorPosition 1 | 2> ..." << endl;
    return true;
}

bool GetMotorPosition(openni::Device& device, vector<string>& command)
{
    if (command.size() < 1)
    {
        printf("Bad command...\n");
        return false;
    }

    XnUInt32 MotorPosition = 0;
    openni::Status rc = device.getProperty(XN_MODULE_PROPERTY_MOTOR_POSITION, &MotorPosition);
    if (openni::STATUS_OK != rc)
    {
        printf("Failed get Motor position, error (%s)\n", openni::OpenNI::getExtendedError());
        return false;
    }
    else
    {
        printf("MotorPosition: %d\n", MotorPosition);
    }

    return true;
}

bool GetMotorStatus(openni::Device& device, vector<string>& command)
{
    if (command.size() < 1)
    {
        printf("Bad command...\n");
        return false;
    }

    XnUInt32 MotorStatus = 0;
    openni::Status rc = device.getProperty(XN_MODULE_PROPERTY_MOTOR_STATUS, &MotorStatus);
    if (openni::STATUS_OK != rc)
    {
        printf("Failed to get motor status, error (%s)\n", openni::OpenNI::getExtendedError());
        return false;
    }
    else
    {
        printf("MotorStatus: %d\n", MotorStatus);
    }

    return true;
}


bool GetMotorTestCount(openni::Device& device, vector<string>& command)
{
    if (command.size() < 1)
    {
        printf("Bad command...\n");
        return false;
    }

    XnUInt32 MotorTestCount = 0;
    openni::Status rc = device.getProperty(XN_MODULE_PROPERTY_MOTOR_TEST_COUNT, &MotorTestCount);
    if (openni::STATUS_OK != rc)
    {
        printf("Failed to get motor test count, error (%s)\n", openni::OpenNI::getExtendedError());
        return false;
    }
    else
    {
        printf("MotorTestCount: %d\n", MotorTestCount);
    }

    return true;
}

bool SetMotorRunTime(openni::Device& device, vector<string>& Command)
{

    if (Command.size() > 1)
    {

        int nValue;
        if (!atoi2(Command[1].c_str(), &nValue))
        {
            printf("Can't understand '%s' as depthIrMode", Command[1].c_str());
            return true;
        }

        if (nValue > 2000 || nValue < 1000)
        {
            printf("input value out of range\n");
            return true;
        }

        openni::Status rc = device.setProperty(XN_MODULE_PROPERTY_MOTOR_RUN_TIME, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("SetMotorRunTime success Run Time param: %d ms \n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <SetMotorRunTime 1000 | 2000 > ..." << endl;
    return true;
}

bool GetMotorRunTime(openni::Device& device, vector<string>& command)
{
    if (command.size() < 1)
    {
        printf("Bad command...\n");
        return false;
    }

    XnUInt32 MotorRunTime = 0;
    openni::Status rc = device.getProperty(XN_MODULE_PROPERTY_MOTOR_RUN_TIME, &MotorRunTime);
    if (openni::STATUS_OK != rc)
    {
        printf("Failed get Motor run time, error (%s)\n", openni::OpenNI::getExtendedError());
        return false;
    }
    else
    {
        printf("MotorRunTime: %d ms\n", MotorRunTime);
    }

    return true;
}

bool GetMotorFeature(openni::Device& device, vector<string>& command)
{
    if (command.size() < 1)
    {
        printf("Bad command...\n");
        return false;
    }

    XnUInt32 MotorFeature = 0;
    openni::Status rc = device.getProperty(XN_MODULE_PROPERTY_MOTOR_FEATURE, &MotorFeature);
    if (openni::STATUS_OK != rc)
    {
        printf("Failed get Motor Feature, error (%s)\n", openni::OpenNI::getExtendedError());
        return false;
    }
    else
    {
        printf("MotorFeature: %d\n", MotorFeature);
    }

    return true;
}

bool GetMotorUpdownState(openni::Device& device, vector<string>& command)
{
    if (command.size() < 1)
    {
        printf("Bad command...\n");
        return false;
    }

    XnUInt32 MotorUpdownState = 0;
    openni::Status rc = device.getProperty(XN_MODULE_PROPERTY_MOTOR_UPDOWN_STATE, &MotorUpdownState);
    if (openni::STATUS_OK != rc)
    {
        printf("Failed get MotorUpdownState, error (%s)\n", openni::OpenNI::getExtendedError());
        return false;
    }
    else
    {
        printf("MotorUpdownState: %d\n", MotorUpdownState);
    }

    return true;
}

bool GetMotorUpdownTime(openni::Device& device, vector<string>& command)
{
    if (command.size() < 1)
    {
        printf("Bad command...\n");
        return false;
    }

    XnUInt32 MotorUpdownTime = 0;
    openni::Status rc = device.getProperty(XN_MODULE_PROPERTY_MOTOR_UPDOWN_TIME, &MotorUpdownTime);
    if (openni::STATUS_OK != rc)
    {
        printf("Failed get MotorUpdownTime, error (%s)\n", openni::OpenNI::getExtendedError());
        return false;
    }
    else
    {
        printf("MotorUpdownTime: %d s\n", MotorUpdownTime);
    }

    return true;
}

bool SetMotorUpdown(openni::Device& device, vector<string>& Command)
{
    if (Command.size() > 1)
    {

        int nValue;
        if (!atoi2(Command[1].c_str(), &nValue))
        {
            printf("Can't understand '%s' as depthIrMode", Command[1].c_str());
            return true;
        }

        if (nValue > 4 || nValue < 0)
        {
            printf("input value out of range\n");
            return true;
        }

        openni::Status rc = device.setProperty(XN_MODULE_PROPERTY_MOTOR_UPDOWN_CONTROL, (XnUInt32)nValue);
        if (rc != openni::STATUS_OK)
        {
            printf("Error: %s\n", openni::OpenNI::getExtendedError());
        }
        else
        {
            printf("SetMotorUpdown success, section param: %d ms \n", nValue);
        }

        return true;
    }

    cout << "Usage: " << Command[0] << " <SetMotorUpdown 0 | 4 > ..." << endl;
    return true;
}


int main(int argc, char **argv)
{
    // Open Device.
    openni::Status rc;

    // don't perform reset on startup
    g_openMode = "R";

    rc = openni::OpenNI::initialize();
    if (rc != openni::STATUS_OK)
    {
        printf("Init failed: %s\n", openni::OpenNI::getExtendedError());
        return 1;
    }

    if (argc > 1 && strcmp(argv[1], "-safe") == 0)
    {
        g_openMode = "RL";
        --argc;
        ++argv;
    }

    openni::Device Device;
    if (!Connect(Device))
    {
        return 2;
    }

    vector<string> DummyCommand;
    PrintVersion(Device, DummyCommand);

    RegisterCB("exit", &byebye, "Exit interactive mode");
    RegisterMnemonic("bye", "exit");

    RegisterCB("set", &SetGeneralParam, "set inner parameters");
    RegisterCB("get", &GetGeneralParam, "get inner parameters");

    RegisterCB("ahb", &GeneralAHB, "read/write the AHB");

    RegisterCB("i2c", &GeneralI2C, "read/write the CMOS");

    RegisterCB("upload", &Upload, "Upload file to flash");
    RegisterCB("download", &Download, "Download file from flash");

    RegisterCB("help", &Help, "Get list of available commands");
    RegisterMnemonic("?", "help");

    RegisterCB("script", &Script, "Run in batch mode");
    RegisterMnemonic("-s", "script");

    RegisterCB("Version", &PrintVersion, "Get version");

    RegisterCB("FileList", &FileList, "Get File List");
    RegisterMnemonic("dir", "FileList");

    RegisterCB("Reset", &Reset, "Reset the device");

    RegisterCB("Delete", &DeleteFile, "Delete File");

    RegisterCB("Log", &Log, "Get Log");

    RegisterCB("Filter", &Filter, "Set/Get Log Filter");

    RegisterCB("Reconnect", &Reconnect, "Close+Open (Init+Shutdown if All)");

    RegisterCB("Sleep", &Sleep, "Sleep some milliseconds. Useful in scripts");

    RegisterCB("Attrib", &Attrib, "Get/Set file attributes");

    RegisterCB("BIST", &RunBIST, "Run BIST");

    RegisterCB("ReadFlash", &ReadFlash, "Reads a chunk of data from the flash");

    RegisterCB("tec", &Tec, "Calibrate/Read TEC");

    RegisterCB("tecfc", &TecFC, "Calibrate/Read TEC");

    RegisterCB("emitter", &Emitter, "Calibrate/Read Emitter");

    RegisterCB("irfloodswitch", &IrfloodSwitch, "IR Flood Switch");

    RegisterCB("irfloodlevel", &IrfloodLevel, "IR Flood Level");

    RegisterCB("projectorFault", &ProjectorFault, "Calibrate ProjectorFault");

    RegisterCB("readfixed", &ReadFixed, "Read Fixed Params");

    RegisterCB("led", &Led, "Set LED state");

    RegisterCB("startread", &StartReadData, "Starts data reading from endpoints");

    RegisterCB("LdpEnable", &LdpEnable, "LDP Enable(on/off)");
    RegisterCB("IrGainSet", &IrGainSet, "irgain set");
    RegisterCB("GetIrGain", &GetIrGain, "get irgain");

    RegisterCB("IrExpSet", &IrExpSet, "irexp set");
    RegisterCB("GetIrExp", &GetIrExp, "get irExp");

    RegisterCB("GetDeviceType", &GetDeviceType, "get device type");
    RegisterCB("GetSerialNumber", &GetSerialNumber, "get serial number");
    RegisterCB("AdoChangeSensor", &AdoChangeSensor, "ado change sensor");

    RegisterCB("SetEccVerifyData", &SetEccVerifyData, "Set EccVerify Data");

    RegisterCB("GetEccVerifyData", &GetEccVerifyData, "get EccVerify Data");

    RegisterCB("GetRandomString", &GetRandomString, "Get Random String");
    RegisterCB("SetRSKey", &SetRSKey, "set rs key");
    RegisterCB("updateFirmware", &updateFirmware, "update firmware");
    RegisterCB("KeepAlive", &KeepAlive, "keep alive");
    RegisterCB("IsSupportLaserSecure", &IsSupportLaserSecure, "is support laser secure");
    RegisterCB("LaserSecureEnable", &LaserSecureEnable, "laser secure enable");
    RegisterCB("GetLaserSecureStatus", &GetLaserSecureStatus, "get laser secure status");
    RegisterCB("LaserCurrentSet", &LaserCurrentSet, "laser current set");
    RegisterCB("GetLaserCurrent", &GetLaserCurrent, "get laser current");
    RegisterCB("SwitchIr", &SwitchIr, "switch left and right ir");
    //RegisterCB("SetCameraParams", &SetCameraParams, "Set Camera Params");
    RegisterCB("getCameraParams", &getCameraParams, "Get Camera Params");
    RegisterCB("SoftReset", &SoftReset, "soft reset");

    RegisterCB("SetRgbAeMode", &SetRgbAeMode, "set rgb Ae Mode");
    RegisterCB("GetRgbAeMode", &GetRgbAeMode, "get rgb Ae Mode");

    RegisterCB("SetCalIrTemp", &SetCalIrTemp, "set cal ir temp");
    RegisterCB("GetCalIrTemp", &GetCalIrTemp, "get cal ir temp");

    RegisterCB("SetCalLdmpTemp", &SetCalLdmpTemp, "set cal ldmp temp");
    RegisterCB("GetCalLdmpTemp", &GetCalLdmpTemp, "get cal ldmp temp");

    RegisterCB("GetRtIrTemp", &GetRtIrTemp, "get realtime ir temp");
    RegisterCB("GetRtLdmpTemp", &GetRtLdmpTemp, "get realtime ldmp temp");

    RegisterCB("SetIrTempCo", &SetIrTemperatureCo, "set ir temperature compensation coefficient");
    RegisterCB("GetIrTempCo", &GetIrTemperatureCo, "get ir  temperature compensation coefficient");

    RegisterCB("SetLdmpTempCo", &SetLdmpTemperatureCo, "set ldmp  temperature compensation coefficient");
    RegisterCB("getLdmpTempCo", &GetLdmpTemperatureCo, "get ldmp  temperature compensation coefficient");

    RegisterCB("TempCompEnable", &TempCompEnable, "Temp CompEnable(on/off)");
    RegisterCB("getTempCompStatus", &GetTempCompStatus, "get temperature compensation status");
    //RegisterCB("updateFirmwareFlash", &updateFirmwareFlash, "update firmware flash chunk");

    //ldp
    RegisterCB("LdpEnable_Register", &LdpEnable_Register, "LDP Enable write Register(on/off)");
    RegisterCB("GetLdpEnable_Register", &GetLdpEnable_Register, "LDP Enable read Register");
    RegisterCB("GetEmitter", &GetEmitterEnable_Register, "Read Emitter status");
    RegisterCB("SetLdpScale", &SetLdpScale, "set ldp scale");
    RegisterCB("GetLdpScale", &GetLdpScale, "get ldp scale");
    RegisterCB("GetLdpStatus", &GetLdpStatus, "get ldp scale");
    RegisterCB("SetLdpThresUp", &SetLdpThresUp, "set ldp thres up value");
    RegisterCB("GetLdpThresUp", &GetLdpThresUp, "get ldp thres up value");
    RegisterCB("SetLdpThresLow", &SetLdpThresLow, "set ldp thres low value");
    RegisterCB("GetLdpThresLow", &GetLdpThresLow, "get ldp thres low value");
    RegisterCB("GetLdpNoise", &GetLdpNoise, "get ldp noise value");
    //QN verify
    RegisterCB("SetFirmwareQN", &SetFirmwareQN, "set firmware QN");
    RegisterCB("GetFirmwareQN", &GetFirmwareQN, "get firmware QN");
    RegisterCB("VerifyFirmwareQN", &VerifyFirmwareQN, "verify firmware QN");

    //get public board version
    RegisterCB("GetPublicBoardVersion", &GetPublicBoardVersion, "get public board version");

    //get mx6300 version
    RegisterCB("VersionMx6300", &PrintVersionMX6300, "Get versionMx6300");

    RegisterCB("SetD2CResolution", &D2CResolutionSet, "set D2CResolution");
    RegisterCB("GetD2CResolution", &GetD2CResolution, "Get D2CResolution");
    RegisterCB("GetUSBSpeed", &GetUSBSpeed, "Get USBSpeed (super/high/full/low)");
    RegisterCB("SetS1SerialNumber", &SetS1SerialNumber, "Set T1/S1 Product SN");
    RegisterCB("GetS1SerialNumber", &GetS1SerialNumber, "Get T1/S1 Product SN");
    RegisterCB("SetDevice_PN", &SetDevice_ProductNumber, "Set KT Product Number");
    RegisterCB("GetDevice_PN", &GetDevice_ProductNumber, "Get KT Product Number");
    RegisterCB("SetAeEnable", &SetAeEnable, "Set Ae enable");
    RegisterCB("GetAeEnable", &GetAeEnableStatus, "Get Ae enable");

    RegisterCB("SetMipiTestEnable", &SetMipiTestEnable, "Set Mipi test enable");
    RegisterCB("GetMipiTestEnable", &GetMipiTestEnableStatus, "Get mipi test enable");

    RegisterCB("SetCfgSnPn", &SetCfgSerialProductNumber, "Set Serial Number and Product Number");
    RegisterCB("GetCfgPn", &GetCfgProductNumber, "Get config product Number");
    RegisterCB("SetReference", &SetReference, "Write reference");
    RegisterCB("SetSwAlignParam", &SetSoftwareAlignParams, "Write software alignment parameters");
    RegisterCB("SetHwAlignParam", &SetHardwareAlignParams, "Write hardware alignment parameters");
    RegisterCB("SetHwDistortionParam", &SetHardwareDistParams, "Write hardware distortion parameters");
    RegisterCB("GetIRModel", &GetIRSensorModel, "Get IR sensor model");
    RegisterCB("GetRgbModel", &GetRgbSensorModel, "Get Rgb sensor model");

    RegisterCB("I2CRead_CameraParam", &I2CReadCameraParam, "use i2c read camera params,only support in mipi project");
    RegisterCB("GetZ0_Baseline", &GetZ0_Baseline, "Get Z0 and baseline");
    RegisterCB("SetZ0_Baseline", &SetZ0_Baseline, "Set Z0 and baseline");
    RegisterCB("GetAEOptions", &GetAEOptions, "Get AE Options");
    RegisterCB("SetAEOptions", &SetAEOptions, "Set AE Options");


    RegisterCB("WriteDistortionParamFlash", &WriteDistortionParamFlash, "Write distortion Param flash");
    RegisterCB("ReadDistortionParamFlash", &ReadDistortionParamFlash, "Read distortion Param flash");
    RegisterCB("DistortionCalEnable", &DistortionCalEnable, "Distortion Cal enable");
    RegisterCB("GetDistortionCalStatus", &GetDistortionCalStatus, "Get Distortion Cal status");
    RegisterCB("GetMipi_CoreBroadId", &GetMipi_CoreBroadId, "Get mipi core broad id");

    RegisterCB("GetPdEnable", &GetPdEnableStatus, "Get pd enable status");
    RegisterCB("SetPdEnable", &SetPdEnableStatus, "Set pd enable status");
    RegisterCB("GetPdAlertEnable", &GetPdAlertStatus, "Get pd alert status");
    RegisterCB("GetPdUpperThreshold", &GetPdUpperThreshold, "Get pd upper threshold");
    RegisterCB("SetPdUpperThreshold", &SetPdUpperThreshold, "Set pd upper threshold");
    RegisterCB("GetPdLowerThreshold", &GetPdLowerThreshold, "Get pd lower threshold");
    RegisterCB("SetPdLowerThreshold", &SetPdLowerThreshold, "Set pd lower threshold");
    RegisterCB("GetPdCurThreshold", &GetPd_CurrentThreshold, "Get pd current threshold");

    RegisterCB("GetBootLoaderPts", &GetBootLoaderPts, "Get bootloader protection status");
    RegisterCB("SetBootLoaderPts", &SetBootLoaderPts, "Get bootloader protection status");

    RegisterCB("SetDepthConfig", &SetDepthConfig, "Set depth config");

    //set laser time 
    RegisterCB("LaserTimeSet", &LaserTimeSet, "laser time set-(write register)");
    RegisterCB("GetLaserTime", &GetLaserTime, "get laser time--(get register)");

    RegisterCB("PostFilterSet", &PostFilterThresholdSet, "post filter threshold (write register)");
    RegisterCB("GetPostFilter", &GetPostFilterThreshold, "post filter threshold (get register)");

    RegisterCB("GetZpps", &GetZpps, "get zpps");
    RegisterCB("IrGainDefSet", &IrGainDefSet, "irgain default set(write flash)");
    RegisterCB("IrExpDefSet", &IrExpDefSet, "irexp default set(write flash)");
    RegisterCB("PostFilterDefSet", &PostFilterThresholdDefSet, "PostFilter default set(write flash)");
    RegisterCB("LaserCurrentDefSet", &LaserCurrentDefSet, "Laser Current default set(write flash)");
    RegisterCB("LaserTimeDefSet", &LaserTimeDefSet, "Laser time default set(write flash)");

    RegisterCB("GetIrGainDef", &GetIrGainDef, "get irgain default (read flash)");
    RegisterCB("GetIrExpDef", &GetIrExpDef, "get irexp default (read flash)");
    RegisterCB("GetPostFilterDef", &GetPostFilterThresholdDef, "get PostFilter default (read flash)");
    RegisterCB("GetLaserCurrentDef", &GetLaserCurrentDef, "Get Laser Current default (read flash)");
    RegisterCB("GetLaserTimeDef", &GetLaserTimeDef, "Laser time default set(read flash)");
    RegisterCB("GetCupVerify", &GetCupVerifyVersion, "get china union pay certification version");

    //TOF
    RegisterCB("SetTofSenEnable", &SetTofSenEnable, "Set ToF sensor enable status");
    RegisterCB("GetTofSenEnable", &GetTofSenEnable, "Get ToF sensor enable status");
    RegisterCB("GetTofSenMea", &GetTofSenMeasure, "Get ToF measuring distance");
    RegisterCB("GetTofSenAppId", &GetTofSenAppId, "Get ToF App id");
    RegisterCB("TofSenCal", &TofSenCalibration, "TOF sensor calibration");
    RegisterCB("SetTofSenAppStart", &SetTofSenAppStart, "Set ToF sensor app start-up enable status");
    RegisterCB("GetTofSenCalParams", &GetTofSenCalibrationParams, "Get TOF sensor calibration params");
    RegisterCB("SetTofSenCalParams", &SetTofSenCalibrationParams, "Set TOF sensor calibration params");

    RegisterCB("DepthIrModeSet", &DepthIrModeSet, " depth ir mode set");
    RegisterCB("GetDepthIrMode", &GetDepthIrMode, "get depth ir mdoe");

    RegisterCB("TecEnable", &TecEnable, " tec enalbe set");

    RegisterCB("setSubtractBackgroudMode", &setSubtractBackgroudMode, "set subtract backgroud mode");
    RegisterCB("getSubtractBackgroudMode", &getSubtractBackgroudMode, "get subtract backgroud state");

    RegisterCB("LoadFirmwareImage", &LoadFirmwareImage, "Load the firmware image");
    RegisterCB("LoadD2CBin", &LoadD2CBin, "Load the D2C bin");
    RegisterCB("LoadDepthBin", &LoadDepthBin, "Load the Depth bin");
    RegisterCB("LoadFilterBin", &LoadFilterBin, "Load the filter bin");
    RegisterCB("LoadRomImage", &LoadRomImage, "Load the ROM image");
    RegisterCB("LoadCommonFile", &LoadCommonFile, "Load the common file");
    RegisterCB("GetCalibration", &GetCalibration, "Get camera calibration data");

    RegisterCB("ExecCmd", &ExecCmd, "Execute serial command");
    RegisterCB("GetDeviceTimestamp", &GetDeviceTimestamp, "Query the current timestamp of device");
    RegisterCB("StartService", &StartService, "Start service of device");

    RegisterCB("GetTOFWorkingMode", &GetTOFWorkingMode, "Getting the working mode of TOF sensor");
    RegisterCB("SetTOFWorkingMode", &SetTOFWorkingMode, "Setting the working mode for TOF sensor");

    RegisterCB("GetSensorID", &GetSensorID, "Getting the sensor ID");
    RegisterCB("GetPlatformVersion", &GetPlatformVersion, "Getting platform version");
    RegisterCB("GetGeneralSerialNumber", &GetGeneralSerialNumber, "Getting the general serial number");
    RegisterCB("SetGeneralSerialNumber", &SetGeneralSerialNumber, "Setting the general serial number");

    RegisterCB("GetEmitterState", &GetEmitterState, "Getting emitter state of TX");
    RegisterCB("SetEmitterState", &SetEmitterState, "Setting emitter state of TX");

    RegisterCB("GetTemperCoeffRX", &GetTemperCoeffRX, "Getting temperature coefficient of RX");
    RegisterCB("SetTemperCoeffRX", &SetTemperCoeffRX, "Setting temperature coefficient of RX");

    RegisterCB("SetMotorTest", &SetMotorTest, "set motor test");
    RegisterCB("GetMotorTestResult", &GetMotorTestResult, "set motor test result");
    RegisterCB("SetMotorPosition", &SetMotorPosition, "set motor position");
    RegisterCB("GetMotorPosition", &GetMotorPosition, "get motor position");
    RegisterCB("GetMotorStatus", &GetMotorStatus, "get motor status");
    RegisterCB("GetMotorTestCount", &GetMotorTestCount, "get motor test count");
    RegisterCB("SetMotorRunTime", &SetMotorRunTime, "set motor run time");
    RegisterCB("GetMotorRunTime", &GetMotorRunTime, "get motor run time");

    RegisterCB("GetMotorFeature", &GetMotorFeature, "get motor feature");
    RegisterCB("GetMotorUpdownState", &GetMotorUpdownState, "get motor updown state");
    RegisterCB("GetMotorUpdownTime", &GetMotorUpdownTime, "get motor updown time");
    RegisterCB("SetMotorUpdown", &SetMotorUpdown, "set motor updown");


    if (argc == 1)
    {
        mainloop(Device, cin, true);
    }
    else
    {
        vector<string> Command;
        for (int i = 1; i < argc; i++)
        {
            Command.push_back(argv[i]);
        }

        for (unsigned int i = 0; i < Command[0].size(); i++)
            Command[0][i] = (char)tolower(Command[0][i]);

        if (Command[0].size() == 0)
        {
            //
        }
        else if (cbs.find(Command[0]) != cbs.end())
        {
            (*cbs[Command[0]])(Device, Command);
        }
        else if (mnemonics.find(Command[0]) != mnemonics.end())
        {
            (*mnemonics[Command[0]])(Device, Command);
        }
        else
        {
            cout << "Unknown command \"" << Command[0] << "\"" << endl;
        }
    }

    g_depthStream.destroy();
    g_colorStream.destroy();
    Device.close();

    return 0;
}
