#ifndef _OB_ONI_DEFINES_H_
#define _OB_ONI_DEFINES_H_


#if (defined _WIN32)
#ifdef OB_ONI_API_EXPORT
#define OB_ONI_API __declspec(dllexport)  
#else
#define OB_ONI_API __declspec(dllimport)
#endif
#elif (__linux__ && (__i386__ || __x86_64__))
#define OB_ONI_API __attribute__ ((visibility("default")))
#endif

#ifndef OB_ONI_MAX_STR
#define OB_ONI_MAX_STR 256
#endif

#ifndef SAFE_DELETE
#define SAFE_DELETE(ptr)   \
    do                     \
    {                      \
        if (NULL != (ptr)) \
        {                  \
            delete (ptr);  \
            (ptr) = NULL;  \
        }                  \
    } while(0)
#endif

#ifndef SAFE_DELETE_ARRAY
#define SAFE_DELETE_ARRAY(ptr) \
    do                         \
    {                          \
        if (NULL != (ptr))     \
        {                      \
            delete[] (ptr);    \
            (ptr) = NULL;      \
        }                      \
    } while(0)
#endif

#ifndef SAFE_FREE
#define SAFE_FREE(ptr)     \
    do                     \
    {                      \
        if (NULL != (ptr)) \
        {                  \
            free(ptr);     \
            (ptr) = NULL;  \
        }                  \
    } while(0)
#endif

#endif //_OB_ONI_DEFINES_H_
