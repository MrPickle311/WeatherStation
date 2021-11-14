#pragma once
#include <map>

namespace Device
{

template <typename ObjectType>
class Singleton
{
public:
    static ObjectType& getInstance()
    {
        static ObjectType object;

        return object;
    }
};

template <typename ObjectType, typename Key>
class Multiton
{
public:
    static ObjectType& get(Key& key)
    {
        if (const auto instance = instances_.find(key);
            instance != instances_.end())
        {
            return instance->second;
        }

        instances_.insert(std::make_pair(key, ObjectType{key}));

        return instances_.at(key);
    }

protected:
    static std::map<Key, ObjectType> instances_;
};

// pointer specalization
template <typename ObjectType, typename Key>
class Multiton<ObjectType, Key*>
{
public:
    static ObjectType& get(Key* key)
    {
        if (const auto instance = instances_.find(key);
            instance != instances_.end())
        {
            return instance->second;
        }

        instances_.insert(std::make_pair(key, ObjectType{key}));

        return instances_.at(key);
    }

protected:
    static std::map<Key*, ObjectType> instances_;
};


template <typename ObjectType, typename Key>
std::map<Key, ObjectType> Multiton<ObjectType, Key>::instances_;

template <typename ObjectType, typename Key>
std::map<Key*, ObjectType> Multiton<ObjectType, Key*>::instances_;

} // namespace Device
