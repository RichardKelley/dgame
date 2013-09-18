#ifndef __dgame_h__
#define __dgame_h__

#include "mvsystem.h"
#include <map>

template<class system_tt> class p1vertex_c;
template<class system_tt> class p2vertex_c;

template<class system_tt>
class p1vertex_c : public vertex_c<system_tt>
{
  public:
    typedef vertex_c<system_tt> vertex_t;
    typedef typename vertex_t::state_t state_t;
    
    p2vertex_c<system_tt>* best_response;
  
    p1vertex_c() : vertex_t(){}
    p1vertex_c(const state_t& s) : vertex_t(s) {} 
};

template<class system_tt>
class p2vertex_c : public vertex_c<system_tt>
{
  public:
    typedef vertex_c<system_tt> vertex_t;
    typedef typename vertex_t::state_t state_t;
    
    set<p1vertex_c<system_tt>* > best_response_of;

    p2vertex_c() : vertex_t(){}
    p2vertex_c(const state_t& s) : vertex_t(s) {} 
};

template<class system_tt, class vertex_tt, class bvertex_tt, class edge_tt, class bedge_tt>
class birrts_c
{
  public:
    typedef system_tt system_t;
    typedef typename system_t::state state;
    typedef typename system_t::cost_t cost_t;
    typedef typename system_t::trajectory trajectory;
    typedef typename system_t::region_t region_t;
    
    rrts_c<vertex_tt, edge_tt> frrts;
    brrts_c<bvertex_tt, bedge_tt> brrts;
    map<cost_t, vertex_tt*> best_response_map;  

    void set_lcmgl(bot_lcmgl_t* lcmgl)
    {
      frrts.lcmgl = lcmgl;
      brrts.lcmgl = lcmgl;
    }
    void set_points_color(float* pc, float width)
    {
      frrts.set_points_color(pc, width);
      brrts.set_points_color(pc, width);
    }
    void set_lines_color(float* lc, float lw)
    {
      frrts.set_lines_color(lc, lw);
      brrts.set_lines_color(lc, lw);
    }
    void set_best_lines_color(float* blc, float blw)
    {
      frrts.set_best_lines_color(blc, blw);
      brrts.set_best_lines_color(blc, blw);
    }

    void insert_rules(vector< pair<size_t, automaton_ss_c> >& psi)
    {
      frrts.system.insert_rules(psi);
      brrts.system.insert_rules(psi);
    }
    
    void initialize(region_t& op_region, region_t& s1, region_t& g1,
        vector<region_t>& regions, bool do_branch_and_bound=true)
    {
      frrts.system.operating_region = op_region;
      frrts.system.goal_region = g1;
      frrts.goal_sample_freq = 0.1;
      frrts.system.insert_regions(regions);
      frrts.initialize(state(s1.c));
      frrts.do_branch_and_bound = do_branch_and_bound;

      brrts.system.operating_region = op_region;
      brrts.system.goal_region = s1;
      brrts.goal_sample_freq = 0.1;
      brrts.system.insert_regions(regions);
      brrts.initialize(state(g1.c));
      brrts.do_branch_and_bound = do_branch_and_bound;
    }

    cost_t get_cost_to_goal(const vertex_tt* v)
    {
      bvertex_tt* nearest_vertex;
      if(!brrts.get_nearest_vertex(v->state, nearest_vertex))
        return nearest_vertex->cost_to_root;
      else
        return cost_t();
    }

    vertex_tt* get_best_vertex()
    {
      vertex_tt* best_vertex = nullptr;
      cost_t best_cost;
      for(auto& pv : frrts.list_vertices)
      {
        cost_t t1 = pv->cost_from_root + get_cost_to_goal(pv);
        if(t1 < best_cost)
        {
          best_cost = t1;
          best_vertex = pv;
        }
      }
      return best_vertex;
    }

    void update_best_response_queue_iter(vertex_tt* pv)
    {
      cost_t t1 = pv->cost_from_root + get_cost_to_goal(pv);
      
      auto map_location = best_response_map.end();
      for(auto it = best_response_map.begin(); it!= best_response_map.end(); it++)
      {
        if(it->second == pv)
          map_location = it;
      }
      
      if(map_location != best_response_map.end())
        best_response_map.erase(map_location);
      best_response_map.insert(make_pair(t1, pv));
      
      //if(best_response_map.size() > 10)
        //best_response_map.erase(--best_response_map.rbegin().base());

      for(auto& pc : pv->children)
        update_best_response_queue_iter(static_cast<vertex_tt*>(pc));
    }

    void update_best_response_queue(vertex_tt* vl, set<vertex_tt*>& rewired_vertices)
    {
      update_best_response_queue_iter(vl);
      for(auto& pv : rewired_vertices)
        update_best_response_queue_iter(static_cast<vertex_tt*>(pv));
    }
};


template<class system_tt>
class dgame_c{
  public:
    typedef system_tt system_t;

    typedef typename system_t::state state;
    typedef typename system_t::trajectory trajectory;
    typedef typename system_t::region_t region_t;

    typedef p1vertex_c<system_tt> p1vertex_t;
    typedef p2vertex_c<system_tt> p2vertex_t;
    typedef bvertex_c<system_tt> bvertex_t;
    typedef edge_c<system_tt> edge_t;
    typedef bedge_c<system_tt> bedge_t;

    typedef typename system_t::cost_t cost_t;

    bot_lcmgl_t* lcmgl;
    birrts_c<system_t, p1vertex_t, bvertex_t, edge_t, bedge_t> p1;
    birrts_c<system_t, p2vertex_t, bvertex_t, edge_t, bedge_t> p2;

    float obstacle_size;

    dgame_c(bot_lcmgl_t* lcmgl_in) {
      lcmgl = lcmgl_in;

      p1.set_lcmgl(lcmgl);
      p2.set_lcmgl(lcmgl);
      
      obstacle_size = 0.5;

      float p2pc[4] = {0,1,0,0.9};
      float p2lc[4] = {0,0,0,0.5};
      float p2blc[4] = {0,1,0,0.9};
      p2.set_points_color(p2pc, 4);
      p2.set_lines_color(p2lc, 1.5);
      p2.set_best_lines_color(p2blc,4);
    }

    void insert_rules(vector< pair<size_t, automaton_ss_c> >& psi){
      p1.insert_rules(psi);
      p2.insert_rules(psi);
    };
   
    void insert_obstacles(vector<region_t>& obstacles)
    {
      p1.frrts.system.obstacle_map.insert_obstacles(obstacles);
      p1.brrts.system.obstacle_map.insert_obstacles(obstacles);
      p2.frrts.system.obstacle_map.insert_obstacles(obstacles);
      p2.brrts.system.obstacle_map.insert_obstacles(obstacles);
    }

    void initialize(region_t& op_region, region_t& s1, region_t& g1, region_t& s2, region_t& g2, 
        vector<region_t>& regions)
    {
      p1.initialize(op_region, s1, g1, regions);
      p2.initialize(op_region, s2, g2, regions, false);
    }
   
    void get_response_trajectory(p2vertex_t& p2v, trajectory& t2)
    {
      p2.frrts.get_trajectory_root(p2v, t2);
      bvertex_t* nearest_vertex;
      if(!p2.brrts.get_nearest_vertex(p2v.state, nearest_vertex))
      {
        trajectory t3;
        p2.brrts.get_trajectory_root(*nearest_vertex, t3);
        t2.append(t3);
      }
      else
      {
        cout<<"224: error"<<endl;
        return;
      }
    }

    bool check_collision_trajectory(const trajectory& t1, const p1vertex_t& p1v, p2vertex_t& p2v)
    {
      trajectory t2;
      get_response_trajectory(p2v, t2);

      bool only_xy = true;
      float collision_distance = obstacle_size;
      int c = 0, cm = min(t1.states.size(), t2.states.size());
      //cout<<"cm: "<< cm << endl;

      while(c < cm)
      {
        auto& s1 = t1.states[c];
        auto& s2 = t2.states[c];
        
        if(s1.dist(s2, only_xy) < collision_distance)
          return true;
        c = c + 10;
      }
      return false;
    }

    void calculate_best_response(p1vertex_t& v)
    {
      p2vertex_t* best_response = p2.frrts.root;
      cost_t best_cost;
      
      trajectory t1;
      p1.frrts.get_trajectory_root(v, t1);
      
      for(auto& p2brm : p2.best_response_map)
      {
        auto pv = p2brm.second;
        if(!check_collision_trajectory(t1, v, *pv))
        {
          cost_t t2 = pv->cost_from_root + p2.get_cost_to_goal(pv);
          if(t2 < best_cost)
          {
            best_cost = t2;
            best_response = pv;
          }
        }
      }
      v.best_response = best_response;
      best_response->best_response_of.insert(&v);
    }
    
    void update_best_response_descendents(p1vertex_t& v)
    {
      calculate_best_response(v);
      for(auto& pc : v.children)
        update_best_response_descendents(*(static_cast<p1vertex_t*>(pc)));
    }
    
    void iteration()
    {
      p1.frrts.iteration();
      p1vertex_t* p1l = p1.frrts.last_added_vertex;
      if(p1l)
        p1.brrts.iteration(&(p1l->state));

      trajectory t1;
      p1vertex_t* p1bv;
      if(p1.frrts.lower_bound_vertex)
        p1bv = p1.frrts.lower_bound_vertex;
      else
        p1bv = p1.get_best_vertex();
      p1.frrts.get_trajectory_root(*p1bv, t1);
      
      set<p2vertex_t*> p2f_rewired_vertices;
      p2.frrts.iteration();
      p2vertex_t* p2l = p2.frrts.last_added_vertex;
      if(p2l)
      {
        p2.brrts.iteration(&(p2l->state));
        p2.update_best_response_queue(p2l, p2f_rewired_vertices);
      }
      
      /* 
      if(p2l)
      {
        vector<p2vertex_t*> S1;
        bool found_p2l = false;
        for(auto& p2brm : p2.best_response_map)
        {
          if(found_p2l)
            S1.push_back(p2brm.second);
          
          if(p2brm.second == p2l)
            found_p2l = true;
        }
        for(auto& pv : S1)
        {
          for(auto& pbrv : pv->best_response_of)
            calculate_best_response(*pbrv);
        }
      }
      if(p1l)
        calculate_best_response(*(static_cast<p1vertex_t*>(p1l)));
      */
    }

    void get_best_trajectories(trajectory& t1, trajectory& t2)
    {
      p2vertex_t* br = nullptr;
      p1vertex_t* p1bv;

      if(p1.frrts.lower_bound_vertex)
        p1bv = p1.frrts.lower_bound_vertex;
      else
        p1bv = p1.get_best_vertex();
      calculate_best_response(*p1bv);
      p1bv->cost_from_root.print(cout, "p1: ", " ");
      
      p1.frrts.get_trajectory_root(*p1bv, t1);
      br = p1bv->best_response;
      if(br)
      {
        cost_t cost_t1 = br->cost_from_root + p2.get_cost_to_goal(br);
        cost_t1.print(cout, "br: ", "\n");
        get_response_trajectory(*br,t2);
      }
    }

    void draw(int iter=-1)
    {
      if(iter < 500)
      {
        p1.frrts.plot_tree();
        p2.frrts.plot_tree();
      }
      /*
      if(p1.frrts.lower_bound_vertex)
        p1.frrts.lower_bound_vertex->cost_from_root.print(cout, "p1: ", " ");
      if(p2.frrts.lower_bound_vertex)
        p2.frrts.lower_bound_vertex->cost_from_root.print(cout, "p2: ", " ");
      cout<<endl;
      */
#if 1
      trajectory t1, t2;
      get_best_trajectories(t1, t2);
      p1.frrts.plot_trajectory(t1, p1.frrts.best_lines_color, p1.frrts.best_lines_width); 
      p2.frrts.plot_trajectory(t2, p2.frrts.best_lines_color, p2.frrts.best_lines_width); 
#endif 
      bot_lcmgl_switch_buffer(lcmgl);
    }
};

#endif
