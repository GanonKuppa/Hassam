#ifndef BASESINGLETON_H
#define BASESINGLETON_H


template <class T>
class Singleton {
public:
	static T& getInstance(){
		static T _Instance;
		return _Instance;
	}

protected:
	Singleton(){};
	virtual ~Singleton(){};

private:
	Singleton(const T& obj);
	Singleton& operator=(const Singleton& obj);

};


#endif
