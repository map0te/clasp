#ifndef CLASP_DEPTH_H_INCLUDED
#define CLASP_DEPTH_H_INCLUDED

#ifdef _MSC_VER
#pragma once
#endif

#include <clasp/constraint.h>
#include <clasp/solver_types.h>
#include <chrono>

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
    // Constraint interface for watches
    PropResult propagate(Solver& s, Literal p, uint32& data);
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
    void computeDist(Solver& s, U32Vec& dist, bool trueOnly);
    void buildAllEdgesReason(Solver& s, LitVec& out);
    void appendTruePath(Solver &s, uint32 node, LitVec& out);
    void setReason(Literal p, const LitVec& reason);
    void updateAssignedEdgesFromTrail(Solver& s);
    static const uint32 INF = UINT32_MAX;

    ExtDepGraph* graph_;
    DepthBindings* data_;
    ReasonStore* nogoods_;

    // Node data
    NodeVec nodes_;              // Per-node depth literals
    U32Vec roots_;               // Static root nodes (when not using declared roots)
    U32Vec maxVal_;              // Max depth value per node
    bool useDeclared_;           // Use declared #root nodes vs static topology
    uint32 nNodes_;

    // Two-distance propagation for incremental value derivation
    U32Vec distLo_;              // Distance using non-false edges (optimistic lower bound)
    U32Vec distHi_;              // Distance using only true edges (pessimistic upper bound)
    U32Vec parent_;              // Parent in BFS tree (for reason construction)
    LitVec parentLit_;           // Edge literal to parent (for reason construction)

    // Incremental dirty tracking
    U32Vec dirtyNodes_;          // Nodes whose distance changed (either distLo or distHi)
    U32Vec prevDistLo_;          // Previous distLo to detect changes
    U32Vec prevDistHi_;          // Previous distHi to detect changes
    bool edgesDirty_;            // Set by watches when edge literals assigned

    // Edge tracking via watches
    typedef PodVector<uint32>::type EdgeIdxVec;
    EdgeIdxVec edgeToIdx_;       // Maps edge literal var to edge index
    LitVec assignedEdgeLits_;    // Cached list of assigned edge literals
    uint32 lastTrailPos_;        // Last trail position processed

    // Reason construction
    LitVec reason_;              // Working buffer for reason construction
    LitVec cachedEdgeReason_;    // Cached all-edges reason (reused across forces)
    U32Vec mark_;                // Marks to avoid duplicate literals in reasons
    uint32 epoch_;               // Current epoch for marking

    // Profiling statistics
    struct {
        uint64_t propagateFixpointCalls = 0;
        uint64_t propagateDepthCalls = 0;
        uint64_t computeDistCalls = 0;
        double totalPropagateTimeMs = 0.0;
        double totalPropagateDepthTimeMs = 0.0;
        double totalPropagateUntilTimeMs = 0.0;
        double totalBfsTimeMs = 0.0;
        double totalEdgeCheckTimeMs = 0.0;
        double totalConstraintCheckTimeMs = 0.0;
        double totalReasonBuildTimeMs = 0.0;
        double totalForceTimeMs = 0.0;
        uint64_t totalForceCalls = 0;
        uint64_t totalLiteralsChecked = 0;
        // Track when literals are forced
        uint64_t forcedCertainPositive = 0;   // Positive forced when dLo==dHi (partial)
        uint64_t forcedCertainNegative = 0;   // Negative forced when dLo==dHi (partial)
        uint64_t forcedBoundNegative = 0;     // Negative forced from bounds (partial)
        uint64_t forcedCompletePositive = 0;  // Positive forced when all edges assigned
        uint64_t forcedCompleteNegative = 0;  // Negative forced when all edges assigned
    } stats_;

    void printStats() const;
};
} // namespace Clasp

#endif