#pragma once

// code taken from https://www.codeproject.com/Articles/313312/Cplusplus11-Lambda-Storage-Without-libcplusplus

#include <assert.h>
#include <stdlib.h>

// LambdaExecutor is an internal class that adds the ability to execute to
// Lambdas. This functionality is separated because it is the only thing
// that needed to be specialized (by return type).

// generateExecutor or receiveExecutor must be called after constructing,
// before use
template<typename T> class LambdaExecutor {};

template <typename Out, typename... In> class LambdaExecutor<Out(In...)> {
  public:
    Out operator()(In ... in)
    {
      assert(lambda != nullptr);
      return executeLambda(lambda, in...);
    }

  protected:
    LambdaExecutor(void *&lambda) : lambda(lambda) {}

    ~LambdaExecutor() {}

    template <typename T> void generateExecutor(T const &lambda)
    {
      executeLambda = [](void *lambda, In... arguments) -> Out
      {
        return ((T *)lambda)->operator()(arguments...);
      };
    }

    void receiveExecutor(LambdaExecutor<Out(In...)> const &other)
    {
      executeLambda = other.executeLambda;
    }

  private:
    void *&lambda;
    Out (*executeLambda)(void *, In...);
};

template <typename... In> class LambdaExecutor<void(In...)> {
  public:
    void operator()(In ... in)
    {
      assert(lambda != nullptr);
      executeLambda(lambda, in...);
    }

  protected:
    LambdaExecutor(void *&lambda) : lambda(lambda) {}

    ~LambdaExecutor() {}

    template <typename T> void generateExecutor(T const &lambda)
    {
      executeLambda = [](void *lambda, In... arguments)
      {
        return ((T *)lambda)->operator()(arguments...);
      };
    }

    void receiveExecutor(LambdaExecutor<void(In...)> const &other)
    {
      executeLambda = other.executeLambda;
    }

  private:
    void *&lambda;
    void (*executeLambda)(void *, In...);
};

// Lambda contains most of the lambda management code and can be used
// directly in external code.
template <typename T> class Lambda {};

template <typename Out, typename ...In> class Lambda<Out(In...)> : 
    public LambdaExecutor<Out(In...)> {
  public:
    Lambda() : LambdaExecutor<Out(In...)>(lambda),
        lambda(nullptr), deleteLambda(nullptr), copyLambda(nullptr)
    {
    }

    Lambda(Lambda<Out(In...)> const &other) : LambdaExecutor<Out(In...)>(lambda),
        lambda(other.copyLambda ? other.copyLambda(other.lambda) : nullptr),
        deleteLambda(other.deleteLambda), copyLambda(other.copyLambda)
    {
      receiveExecutor(other);
    }

    template<typename T>
    Lambda(T const &lambda) : LambdaExecutor<Out(In...)>(this->lambda), lambda(nullptr)
    {
      // Copy should set all variables
      copy(lambda);
    }

    ~Lambda()
    {
      if (deleteLambda != nullptr) deleteLambda(lambda);
    }

    Lambda<Out(In...)> &operator =(Lambda<Out(In...)> const &other)
    {
      this->lambda = other.copyLambda ? other.copyLambda(other.lambda) : nullptr;
      receiveExecutor(other);
      this->deleteLambda = other.deleteLambda;
      this->copyLambda = other.copyLambda;
      return *this;
    }

    template<typename T> Lambda<Out(In...)> &operator =(T const &lambda)
    {
      copy(lambda);
      return *this;
    }

    operator bool()
    {
      return lambda != nullptr;
    }

  private:
    template<typename T>
    void copy(T const &lambda)
    {
      if (this->lambda != nullptr) deleteLambda(this->lambda);
      this->lambda = new T(lambda);
      //this->lambda = new void*((void*)lambda);

      generateExecutor(lambda);

      deleteLambda = [](void *lambda)
      {
        delete (T *)lambda;
        //delete lambda;
      };

      copyLambda = [](void *lambda) -> void *
      {
        return lambda ? new T(*(T *)lambda) : nullptr;
        //return lambda ? new void*(lambda) : nullptr;
      };
    }

    void *lambda;
    void (*deleteLambda)(void *);
    void *(*copyLambda)(void *);
};
