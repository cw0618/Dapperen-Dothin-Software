#include "mx6000.h"

mx6000 * mx6000::_instance = nullptr;

mx6000* mx6000::Instance()
{
    if(nullptr == _instance)
    {
        _instance = new mx6000();
    }

    return _instance;

}
