#ifndef CLASP_DEPTH_H_INCLUDED
#define CLASP_DEPTH_H_INCLUDED

#ifdef _MSC_VER
#pragma once
#endif

#include <clasp/constraint.h>
#include <clasp/solver_types.h>

namespace Clasp {
class Solver;
class SharedContext;
class ExtDepGraph;

class DepthBindings {
  public:
    struct Binding {
        uint32 node;
        int32 depth;
        Literal lit;
    };
    struct Root {
        uint32 node;
        Literal lit;
    };
    typedef PodVector<Binding>::type BindingVec;
    typedef PodVector<Root>::type RootVec;
    DepthBindings () {};
    void add(uint32 node, int32 depth, Literal lit) { bindings.push_back(Binding{node, depth, lit}); }
    void addRoot(uint32 node, Literal lit) { roots.push_back(Root{node, lit}); }
    bool empty() const { return bindings.empty(); }
    BindingVec bindings;
    RootVec roots;
};

class DepthPropagator : public PostPropagator {
  public:
    enum { PRIO = PostPropagator::priority_reserved_ufs + 2}; // Acyclicity check is + 1
    DepthPropagator(ExtDepGraph* graph, DepthBindings* bindings);
    ~DepthPropagator();
    // PostPropagator
    uint32 priority() const { return static_cast<uint32>(PRIO); }
    bool init(Solver& s);
    void reset();
    bool propagateFixpoint(Solver& s, PostPropagator* ctx);
    bool isModel(Solver& s);
    bool valid(Solver& s);
    void destroy(Solver* s, bool detach);
  private:
    DepthPropagator(const DepthPropagator&);
    DepthPropagator& operator=(const DepthPropagator&);
    struct ReasonStore;
    struct DepthLit { int32 depth; Literal lit; };
    typedef PodVector<DepthLit>::type DepthLitVec;
    typedef PodVector<DepthLitVec>::type NodeVec;
    typedef PodVector<uint32>::type U32Vec;

    void reason(Solver& s, Literal p, LitVec& out);
    bool propagateDepth(Solver &s);
    void computeDist(Solver& s, U32Vec& dist, bool trueOnly, bool withParents);
    void buildLowerReason(Solver& s, LitVec& out);
    void appendTruePath(Solver &s, uint32 node, LitVec& out);
    void setReason(Literal p, const LitVec& reason);
    static const uint32 INF = UINT32_MAX;

    ExtDepGraph* graph_;
    DepthBindings* data_;
    ReasonStore* nogoods_;
    NodeVec nodes_;
    U32Vec roots_;
    U32Vec maxVal_;
    U32Vec distLo_;
    U32Vec distHi_;
    U32Vec parent_;
    LitVec parentLit_;
    LitVec reason_;
    U32Vec mark_;
    uint32 epoch_;
    uint32 nNodes_;
    bool   useDeclared_;
};
} // namespace Clasp

#endif