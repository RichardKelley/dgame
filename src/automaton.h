#ifndef __automaton_h__
#define __automaton_h__

#include "utils.h"
#include "driving.h"

class label_c{
  public:
    const static size_t len = 8;
    symbol s;

    label_c() {};
    label_c(symbol s_) : s(s_){
    }
    void insert(const size_t w){
      s = s | (1<<w);
    }
    void insert(const label_c& l){
      s = s | l.s;
    }
    void remove(const label_c& l){
      s = s ^ l.s;
    }
    void remove(const size_t w){
      s = s ^ (1<<w);
    }
    label_c minus(const label_c& l) const{
      return label_c(s ^ l.s);
    }
    label_c intersect(const label_c& l) const{
      return label_c(s & l.s);
    }
    bool is_subset(const label_c& l) const{
      return !!(s & l.s);
    }
    friend bool operator<(const label_c& a, const label_c& b){
      return a.s < b.s;
    }
    bool operator[](const size_t w){
      return (s & (1<<w));
    }
    bool operator!=(label_c& l){
      return (s != l.s);
    }
    void print(){
      for(int i=7; i>=0; i--)
        cout<< !!(s & (1<<i));
      cout<<endl;
    }
};

template<size_t dim>
class cost_c{
  public:
    vector<float> c;

    cost_c(){
      c = vector<float>(dim,0);
    }
    cost_c(vector<float>& c_) : c(c_) {}
    cost_c(const cost_c<dim>& c2){
      c = c2.c;
    }
    cost_c operator+(const cost_c& c2){
      cost_c toret;
      toret += c2;
      return toret;
    }
    cost_c& operator+=(const cost_c& c2){
      for(size_t i=0; i<dim; i++)
        c[i] += c2.c[i];
      return *this;
    }
    bool operator<(const cost_c& c2){
      for(size_t i=0; i<dim; i++){
        if(c[i] < c2.c[i])
          return true;
        else if(c[i] > c2.c[i])
          return false;
      }
      return true;
    }
    void print(){
      cout<<"(";
      for(auto& ci : c)
        cout<<ci<<",";
      cout<<")"<<endl;
    }
};

class timed_word_c{
  public:
    float dt;
    label_c label;

    timed_word_c(float dt_, label_c label_) : dt(dt_), label(label_){
    }
    timed_word_c(const timed_word_c& tw){
      dt = tw.dt;
      label = tw.label;
    }
    void print(){
      cout<<"tw: "<< dt<<" ";
      label.print();
    }
};

class automaton_ss_c{
  public:
    bool is_positive;
    label_c label;
    float weight;
    float cost;

    automaton_ss_c(){}
    automaton_ss_c(bool is_positive_, label_c label_,
        float weight_, float cost_):
      is_positive(is_positive_), label(label_), weight(weight_), cost(cost_){
      }

    float get_cost(const timed_word_c& tw) const{
      if( (label.is_subset(tw.label) && is_positive) ||
          (!label.is_subset(tw.label) && !is_positive))
        return 0;
      else
        return cost + weight*tw.dt;
    }

    friend bool operator<(const automaton_ss_c& psi1, const automaton_ss_c& psi2){
      return psi1.label < psi2.label;
    }
};

template<size_t max_priority>
class automaton_product{
  public:
    set<pair<size_t, automaton_ss_c> > rules;

    int insert(const automaton_ss_c& psi, size_t p){
      assert(p <= max_priority);
      auto pp = make_pair(p, psi);
      rules.insert(pp);
      return 0;
    };

    cost_c<max_priority+2> get_cost(const timed_word_c& tw){
      cost_c<max_priority+2> cost;
      for(auto& ppsi : rules){
        auto& psi = ppsi.second;
        float t1 = psi.get_cost(tw);
        cost.c[ppsi.first] += t1;  
      }
      cost.c[max_priority+1] = tw.dt;
      return cost;
    };
};
#endif
