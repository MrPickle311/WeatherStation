#pragma once
#include <map>

namespace Device
{

template<typename ObjectType>
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
	static ObjectType& get(const Key& key)
	{
		if (const auto instance = instances_.find(key);
        	instance != instances_.end())
		{
			return instance->second;
		}

		auto instance {ObjectType{}};
		instances_[key] = instance;

		return instance;
  }
private:
	static std::map<Key, ObjectType> instances_;
};

}

