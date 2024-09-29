#ifndef XN_DOTHIN_STATUS
#define XN_DOTHIN_STATUS

#include <XnStatus.h>
#include "imagekit.h"


/** Returns Y if X isn't DT_ERROR_OK. */
#define XN_IS_DOTHINSTATUS_OK_RET(x, y)	\
		if (x == DT_ERROR_FAILED)		\
		{							\
			return XN_STATUS_ERROR; \
		}                           \
		else if (x != DT_ERROR_OK)  \
		{                           \
			return (y);             \
		}                           \
        else{                       \
           x = XN_STATUS_OK;        \
        }

/** Returns X if X isn't DT_ERROR_OK. */
#define XN_IS_DOTHINSTATUS_OK(x)			\
		XN_IS_DOTHINSTATUS_OK_RET(x, x)



#endif  //XN_DOTHIN_STATUS