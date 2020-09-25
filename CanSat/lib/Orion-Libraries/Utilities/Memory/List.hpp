#ifndef __ORION_UTILITIES_MEMORY_LIST_H__
#define __ORION_UTILITIES_MEMORY_LIST_H__

#include "Orion.h"

#include <stdint.h>

namespace Orion
{
    namespace Utilities
    {
        namespace Memory
        {
            template<typename T>
            class List
            {
                private:
                    T* _list = nullptr;
                    bool _initialized = false;
                    uint32_t _size = NAN, _used = NAN;

                    bool Resize(uint32_t newSize)
                    {
                        if (!_initialized)
                            return false;

                        T* old = _list;
                        _list = new T[newSize];
                        
                        if (!_list)
                        {
                            _list = old;
                            return false;
                        }

                        memcpy((void*)_list, (const void*)old, (_size < newSize) ? _size : newSize);
                        delete old;

                        return true;
                    }
                public:
                    List()
                    {
                        _list = new T[1];
                        if (!_list)
                            return;
                        _initialized = true;
                        _size = 1;
                        _used = 0;
                    }
                    List(uint32_t size)
                    {
                        _list = new T[size];
                        if (!_list)
                            return;
                        _initialized = true;
                        _size = size;
                        _used = 0;
                    }

                    bool Add(T& element)
                    {
                        if (_size == _used)
                            if (!Resize(_size * 2))
                                return false;
                        _list[_used++] = element;
                        return true;
                        
                    }

                    void Clear()
                    {
                        if (!_initialized)
                            return;

                        delete _list;
                        _list = new T[1];
                        if (!_list)
                            return;
                        _initialized = true;
                        _size = 1;
                        _used = 0;
                    }

                    bool Exist(T& element)
                    {
                        if (!_initialized)
                            return false;

                        for (uint32_t i = 0; i < _used; i++)
                            if (_list[i] == element)
                                return true;

                        return false;
                    }
                    
                    uint32_t GetCapacity()
                    {
                        return _size;
                    }
                    
                    uint32_t GetSize()
                    {
                        return _used;
                    }

                    bool RemoveAt(uint32_t position)
                    {
                        if (!_initialized)
                            return false;
                        memcpy((void*)(_list + position), (void*)(_list + position + 1), (_used - position - 1) * sizeof(T));
                        _used--;
                        return true;
                    }

                    T& operator[](uint32_t position)
                    {
                        if (!_initialized || _used < position)
                        {
                            T ret = T();
                            return ret;
                        }

                        return _list[position];
                    }

                    ~List()
                    {
                        if (!_initialized)
                            return;
                        delete _list;
                        _initialized = false;
                        _size = _used = NAN;
                    }
            };
        }
    }
}

#endif//__ORION_UTILITIES_MEMORY_LIST_H__