/***************************************************************************/
/* */
/* Copyright (c) 2013-2021 Orbbec 3D Technology, Inc */
/* 奥比中光科技有限公司 版权所有 2013-2031 */
/* */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts */
/* the terms of the license. */
/* */
/* 本软件文档资料是奥比中光科技有限公司的资产, 任何人士阅读和使用本资料必须获得 */
/* 相应的书面授权, 承担保密责任和接受相应的法律约束. */
/* */
/***************************************************************************/
#ifndef MININI_H
#define MININI_H

//#include "minGlue.h"
#include <stdio.h>

#define INI_FILETYPE                    FILE*
#define ini_openread(filename,file)     ((*(file) = fopen((filename),"rb")) != NULL)
#define ini_openwrite(filename,file)    ((*(file) = fopen((filename),"wb")) != NULL)
#define ini_openrewrite(filename,file)  ((*(file) = fopen((filename),"r+b")) != NULL)
#define ini_close(file)                 (fclose(*(file)) == 0)
#define ini_read(buffer,size,file)      (fgets((buffer),(size),*(file)) != NULL)
#define ini_write(buffer,file)          (fputs((buffer),*(file)) >= 0)
#define ini_rename(source,dest)         (rename((source), (dest)) == 0)
#define ini_remove(filename)            (remove(filename) == 0)

#define INI_FILEPOS                     long int
#define ini_tell(file,pos)              (*(pos) = ftell(*(file)))
#define ini_seek(file,pos)              (fseek(*(file), *(pos), SEEK_SET) == 0)

/* for floating-point support, define additional types and functions */
#define INI_REAL                        float
#define ini_ftoa(string,value)          sprintf((string),"%f",(value))
#define ini_atof(string)                (INI_REAL)strtod((string),NULL)


#if (defined _UNICODE || defined __UNICODE__ || defined UNICODE) && !defined INI_ANSIONLY
//  #include <tchar.h>
  #define mTCHAR char
#else
  /* force TCHAR to be "char", but only for minIni */
  #define mTCHAR char
#endif

#if !defined INI_BUFFERSIZE
  #define INI_BUFFERSIZE  512
#endif

#if defined __cplusplus
  extern "C" {
#endif

int   ini_getbool(const mTCHAR *Section, const mTCHAR *Key, int DefValue, const mTCHAR *Filename);
long  ini_getl(const mTCHAR *Section, const mTCHAR *Key, long DefValue, const mTCHAR *Filename);
int   ini_gets(const mTCHAR *Section, const mTCHAR *Key, const mTCHAR *DefValue, mTCHAR *Buffer, int BufferSize, const mTCHAR *Filename);
int   ini_getsection(int idx, mTCHAR *Buffer, int BufferSize, const mTCHAR *Filename);
int   ini_getkey(const mTCHAR *Section, int idx, mTCHAR *Buffer, int BufferSize, const mTCHAR *Filename);

#if defined INI_REAL
INI_REAL ini_getf(const mTCHAR *Section, const mTCHAR *Key, INI_REAL DefValue, const mTCHAR *Filename);
#endif

#if !defined INI_READONLY
int   ini_puti(const mTCHAR *Section, const mTCHAR *Key, int Value, const mTCHAR *Filename);
int   ini_putl(const mTCHAR *Section, const mTCHAR *Key, long Value, const mTCHAR *Filename);
int   ini_puts(const mTCHAR *Section, const mTCHAR *Key, const mTCHAR *Value, const mTCHAR *Filename);
#if defined INI_REAL
int   ini_putf(const mTCHAR *Section, const mTCHAR *Key, INI_REAL Value, const mTCHAR *Filename);
#endif
#endif /* INI_READONLY */

#if !defined INI_NOBROWSE
typedef int (*INI_CALLBACK)(const mTCHAR *Section, const mTCHAR *Key, const mTCHAR *Value, void *UserData);
int  ini_browse(INI_CALLBACK Callback, void *UserData, const mTCHAR *Filename);
#endif /* INI_NOBROWSE */

#if defined __cplusplus
  }
#endif


#if defined __cplusplus

#if defined __WXWINDOWS__
	#include "wxMinIni.h"
#else
  #include <string>

  /* The C++ class in minIni.h was contributed by Steven Van Ingelgem. */
  class minIni
  {
  public:
    minIni(const std::string& filename) : iniFilename(filename)
      { }

    bool getbool(const std::string& Section, const std::string& Key, bool DefValue=false) const
      { return ini_getbool(Section.c_str(), Key.c_str(), int(DefValue), iniFilename.c_str()) != 0; }

    long getl(const std::string& Section, const std::string& Key, long DefValue=0) const
      { return ini_getl(Section.c_str(), Key.c_str(), DefValue, iniFilename.c_str()); }

    int geti(const std::string& Section, const std::string& Key, int DefValue=0) const
      { return static_cast<int>(this->getl(Section, Key, long(DefValue))); }

    std::string gets(const std::string& Section, const std::string& Key, const std::string& DefValue="") const
      {
        char buffer[INI_BUFFERSIZE];
        ini_gets(Section.c_str(), Key.c_str(), DefValue.c_str(), buffer, INI_BUFFERSIZE, iniFilename.c_str());
        return buffer;
      }

    std::string getsection(int idx) const
      {
        char buffer[INI_BUFFERSIZE];
        ini_getsection(idx, buffer, INI_BUFFERSIZE, iniFilename.c_str());
        return buffer;
      }

    std::string getkey(const std::string& Section, int idx) const
      {
        char buffer[INI_BUFFERSIZE];
        ini_getkey(Section.c_str(), idx, buffer, INI_BUFFERSIZE, iniFilename.c_str());
        return buffer;
      }

#if defined INI_REAL
    INI_REAL getf(const std::string& Section, const std::string& Key, INI_REAL DefValue=0) const
      { return ini_getf(Section.c_str(), Key.c_str(), DefValue, iniFilename.c_str()); }
#endif

#if ! defined INI_READONLY
    bool put(const std::string& Section, const std::string& Key, long Value) const
      { return ini_putl(Section.c_str(), Key.c_str(), Value, iniFilename.c_str()) != 0; }

    bool put(const std::string& Section, const std::string& Key, int Value) const
      { return ini_putl(Section.c_str(), Key.c_str(), (long)Value, iniFilename.c_str()) != 0; }

    bool put(const std::string& Section, const std::string& Key, bool Value) const
      { return ini_putl(Section.c_str(), Key.c_str(), (long)Value, iniFilename.c_str()) != 0; }

    bool put(const std::string& Section, const std::string& Key, const std::string& Value) const
      { return ini_puts(Section.c_str(), Key.c_str(), Value.c_str(), iniFilename.c_str()) != 0; }

    bool put(const std::string& Section, const std::string& Key, const char* Value) const
      { return ini_puts(Section.c_str(), Key.c_str(), Value, iniFilename.c_str()) != 0; }

#if defined INI_REAL
    bool put(const std::string& Section, const std::string& Key, INI_REAL Value) const
      { return ini_putf(Section.c_str(), Key.c_str(), Value, iniFilename.c_str()) != 0; }
#endif

    bool del(const std::string& Section, const std::string& Key) const
      { return ini_puts(Section.c_str(), Key.c_str(), 0, iniFilename.c_str()) != 0; }

    bool del(const std::string& Section) const
      { return ini_puts(Section.c_str(), 0, 0, iniFilename.c_str()) != 0; }
#endif

  private:
    std::string iniFilename;
  };

#endif /* __WXWINDOWS__ */
#endif /* __cplusplus */

#endif /* MININI_H */
