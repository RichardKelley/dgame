#include "smpl/smpl.h"

template<size_t N>
class mvmap_c : public map_c<N>
{
  public:
    vector< mvregion_c<N> > obstacles;

    mvmap_c() : map_c<N> () {}
    void insert_obstacles(vector< mvregion_c<N> >& obstacles_in)
    {
      obstacles = obstacles_in;
    }
    bool is_in_collision(const float s[N])
    {
      for(auto& r : obstacles)
      {
        if(r.is_inside(s))
        {
          //cout<<"returning true"<<endl;
          return true;
        }
      }
      return false;
    }
};
