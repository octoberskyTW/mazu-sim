/// \file singleton.hpp
///
/// \date 2020/01/08
///
/// \author Chun-Hsu Lai

#ifndef SINGLETON_HPP_
#define SINGLETON_HPP_

template<class T>
class Singleton {
  public:
		static T* Instance() {
			return &GetInstance();
		}
        Singleton(const Singleton &other) = delete;
        Singleton &operator=(const Singleton &other) = delete;
  protected:
		Singleton(){};
		virtual ~Singleton(){};
	
  private:
		static T &GetInstance() {
			static T single;
			return static_cast<T&>(single);
		}

};

#endif // SINGLETON_HPP_