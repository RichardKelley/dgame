#ifndef __dgame_h__
#define __dgame_h__

#include "mvsystem.h"
#include <map>
#include "mvmap.h"

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
      frrts.goal_sample_freq = 0.25;
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

    void update_best_response_map()
    {
      best_response_map.clear();
      for(auto p2v : frrts.list_vertices)
        update_best_response_queue_iter(p2v, true);
    }

    void update_best_response_queue_iter(vertex_tt* pv, bool dont_check_if_exists=false)
    {
      if(!frrts.system.is_in_goal(pv->state))
        return;

      cost_t t1 = pv->cost_from_root;
      if(!dont_check_if_exists)
      {
        auto map_location = best_response_map.end();
        for(auto it = best_response_map.begin(); it!= best_response_map.end(); it++)
        {
          if(it->second == pv)
            map_location = it;
        }

        if(map_location != best_response_map.end())
          best_response_map.erase(map_location);
      }
      best_response_map.insert(make_pair(t1, pv));
      
      //if(best_response_map.size() > 10)
        //best_response_map.erase(--best_response_map.rbegin().base());

      //for(auto& pc : pv->children)
        //update_best_response_queue_iter(static_cast<vertex_tt*>(pc));
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

    lcm_t* lcm;
    bot_lcmgl_t* lcmgl;
    birrts_c<system_t, p1vertex_t, bvertex_t, edge_t, bedge_t> p1;
    birrts_c<system_t, p2vertex_t, bvertex_t, edge_t, bedge_t> p2;

    float obstacle_size;

    dgame_c(lcm_t* lcm_in, bot_lcmgl_t* lcmgl_in) {
      lcm = lcm_in;
      lcmgl = lcmgl_in;

      p1.set_lcmgl(lcmgl);
      p2.set_lcmgl(lcmgl);
      
      obstacle_size = 4;

      float p2pc[4] = {0,1,0,0.9};
      float p2lc[4] = {0,0,0,0.5};
      float p2blc[4] = {0,1,0,0.9};
      p2.set_points_color(p2pc, 4);
      p2.set_lines_color(p2lc, 2);
      p1.set_lines_color(p1.frrts.lines_color, 2);
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
    
    void insert_heuristic_regions(vector<region_t>& regions)
    {
      p1.frrts.system.heuristic_sampling_regions = regions;
      p2.frrts.system.heuristic_sampling_regions = regions;
      p1.brrts.system.heuristic_sampling_regions = regions;
      p2.brrts.system.heuristic_sampling_regions = regions;
    }

    void initialize(region_t& op_region, region_t& s1, region_t& g1, region_t& s2, region_t& g2, 
        vector<region_t>& regions)
    {
      p1.initialize(op_region, s1, g1, regions);
      p2.initialize(op_region, s2, g2, regions);
    }
   
    void get_response_trajectory(p2vertex_t& p2v, trajectory& t2)
    {
      p2.frrts.get_trajectory_root(p2v, t2);
    }

    void calculate_best_response(p1vertex_t& v)
    {
      p2vertex_t* best_response = NULL;
      
      trajectory t1, t2;
      p1.frrts.get_trajectory_root(v, t1);
      
      for(auto& p2brm : p2.best_response_map)
      {
        auto pv = p2brm.second;
        get_response_trajectory(*pv, t2);
        if(!p2.frrts.check_collision_trajectory(t1, t2, obstacle_size))
        {
          best_response = pv;
        }
      }
      v.best_response = best_response;
      if(best_response)
        best_response->best_response_of.insert(&v);
    }
    
    void update_best_response_descendents(p1vertex_t& v)
    {
      calculate_best_response(v);
      for(auto& pc : v.children)
        update_best_response_descendents(*(static_cast<p1vertex_t*>(pc)));
    }

#if 0 
    void iteration()
    {
      p1.frrts.iteration();
      p1vertex_t* p1l = p1.frrts.last_added_vertex;
      //if(p1l)
        //p1.brrts.iteration(&(p1l->state));

      trajectory t1;
      p1vertex_t* p1bv;
      if(p1.frrts.lower_bound_vertex)
      {
        p1bv = p1.frrts.lower_bound_vertex;
        p1.frrts.get_trajectory_root(*p1bv, t1);
      }
      //else
        //p1bv = p1.get_best_vertex();
      
      set<p2vertex_t*> p2f_rewired_vertices;
      p2.frrts.iteration(NULL, &p2f_rewired_vertices, &t1, obstacle_size);
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
#else

    void delete_colliding_vertices(p2vertex_t* pv, trajectory& t1)
    {
      trajectory t2;
      auto pvci = pv->children.begin();
      while(pvci != pv->children.end())
      {
        p2vertex_t* pvc = static_cast<p2vertex_t*>(*pvci);

        p2.frrts.system.extend_to(pv->state, pvc->state, 
            false, t2, pvc->edge_from_parent->opt_data);
        t2.t0 = pv->t0;

        if(p2.frrts.check_collision_trajectory(t1, t2, obstacle_size))
        {
          p2.frrts.delete_downstream(*pvc);
          pv->children.erase(pvci++);
        }
        else
        {
          delete_colliding_vertices(pvc, t1);
          ++pvci;
        }
      }
    }
    
    void iteration()
    {
      p1.frrts.iteration();
      p1vertex_t* p1l = p1.frrts.last_added_vertex;
      
      trajectory t1;
      set<p2vertex_t*> p2f_rewired_vertices;
      p1vertex_t* p1bv;
      if(p1.frrts.lower_bound_vertex)
      {
        p1bv = p1.frrts.lower_bound_vertex;
        p1.frrts.get_trajectory_root(*p1bv, t1);
      
        p2.frrts.iteration(NULL, &p2f_rewired_vertices, &t1, obstacle_size);
      }
      else
      {
        p2.frrts.iteration(NULL, &p2f_rewired_vertices);
      }

      p2vertex_t* p2l = p2.frrts.last_added_vertex;
      if(p2l)
        p2.update_best_response_queue(p2l, p2f_rewired_vertices);

      if(p1bv)
      {
        delete_colliding_vertices(p2.frrts.root, t1);
        p2.frrts.update_all_costs();
        p2.update_best_response_map();
      }
    }
#endif

    void get_best_trajectories(trajectory& t1, trajectory& t2)
    {
      p2vertex_t* br = nullptr;
      p1vertex_t* p1bv = p1.frrts.root;

      if(p1.frrts.lower_bound_vertex)
      {
        p1bv = p1.frrts.lower_bound_vertex;
        p1.frrts.get_trajectory_root(*p1bv, t1);

        calculate_best_response(*p1bv);
        br = p1bv->best_response;
        if(br)
        {
          cost_t cost_t1 = br->cost_from_root;
          cost_t1.print(cout, "\tbr: ", "\n");
          get_response_trajectory(*br,t2);
        }
      }
    }

    // Transform the state ( x, y, theta) to 
    // the pose ( (x,y,z) , ( q_0, q_1, q_2, q_3) ), 
    // where q = ( q_0, q_1, q_2, q_3) is a quaternion 
    int create_pose_from_state(float* x, bot_core_pose_t& pose) {

      double rpy[3] = {0, 0.0, x[2]};
      double orientation[4];
      bot_roll_pitch_yaw_to_quat( rpy, orientation);

      struct timeval tv_now;
      gettimeofday( &tv_now, NULL);
      pose.utime = tv_now.tv_sec*1e6 + tv_now.tv_usec;

      pose.pos[0] = x[0];
      pose.pos[1] = x[1];
      pose.pos[2] = 0;

      for(int i = 0; i < 4; i++)
        pose.orientation[i] = orientation[i];

      return 0;
    }
    
    void draw_tree(int iter=-1)
    {
      if(iter < 500)
      {
        //p1.frrts.plot_tree();
        p2.frrts.plot_tree();
      }
      cout<<"lb cost:"<<endl;
      if(p1.frrts.lower_bound_vertex)
        p1.frrts.lower_bound_vertex->cost_from_root.print(cout, "\tp1: ", "--");
      if(p2.frrts.lower_bound_vertex)
        p2.frrts.lower_bound_vertex->cost_from_root.print(cout, "\tp2: ", "\n");
      cout<<endl;

      trajectory t1, t2;
      get_best_trajectories(t1, t2);
      p1.frrts.plot_trajectory(t1, p1.frrts.best_lines_color, p1.frrts.best_lines_width); 
      p2.frrts.plot_trajectory(t2, p2.frrts.best_lines_color, p2.frrts.best_lines_width); 
#if 1
      int index = 0;
      if(iter > 1000)
      {
        index = 400;
      }
      bot_core_pose_t pose;
      if(t1.states.size())
        create_pose_from_state(t1.states[index].x, pose);
      bot_core_pose_t_publish(lcm, "POSE1", &pose);

      if(t2.states.size())
        create_pose_from_state(t2.states[index].x, pose);
      bot_core_pose_t_publish(lcm, "POSE2", &pose);
#endif
    }
};

#endif
