#include <clasp/depth.h>
#include <clasp/solver.h>
#include <clasp/shared_context.h>
#include <clasp/dependency_graph.h>
#include <algorithm>

namespace Clasp {

// DepthPropagator: Two-distance incremental propagation
//
// Approach:
//   - Compute TWO distances per node:
//     * distLo: using non-false edges (optimistic - shortest possible path)
//     * distHi: using only true edges (pessimistic - current best path)
//   - When distLo == distHi: depth is CERTAIN → force exact value (matches ASP #min)
//   - When distLo != distHi: depth uncertain → only force bounds
//   - Reason clauses include ALL assigned edges (both true and false)
//
// Optimizations:
//   - Watch-based dirty tracking: only propagate when edge literals change
//   - Incremental edge tracking: cache assigned edges from solver trail
//   - Dirty node tracking: only check nodes whose distance changed
//   - Cached edge reasons: build once per propagation, reuse for all forces
//   - Complete profiling for performance analysis

struct DepthPropagator::ReasonStore {
	typedef PodVector<LitVec*>::type NogoodMap;
	NogoodMap db; 
	void getReason(Literal p, LitVec& out) {
		if (p.var() < db.size()) {
			if (const LitVec* r = db[p.var()]) {
				out.insert(out.end(), r->begin(), r->end());
			}
		}
	}
	void setReason(Literal p, const LitVec& reason) {
		Var v = p.var();
		if (v >= db.size()) { db.resize(v + 1, 0); }
		if (db[v] == 0)     { db[v] = new LitVec(reason); }
		else                { *db[v] = reason; }
	}
	~ReasonStore() { std::for_each(db.begin(), db.end(), DeleteObject()); }
};

const uint32 DepthPropagator::INF;

DepthPropagator::DepthPropagator(ExtDepGraph* graph, DepthBindings* data)
    : graph_(graph), data_(data), nogoods_(0), nNodes_(0), useDeclared_(false),
      edgesDirty_(false), lastTrailPos_(0), epoch_(0) {}

DepthPropagator::~DepthPropagator() {
	printStats();
	delete nogoods_;
}

bool DepthPropagator::init(Solver& s) {
	if (!graph_ || !data_ || data_->empty()) { graph_ = 0; return true; }
	nNodes_ = graph_->nodes();
	for (DepthBindings::BindingVec::const_iterator it = data_->bindings.begin(), end = data_->bindings.end(); it != end; ++it) {
		nNodes_ = std::max(nNodes_, it->node + 1);
	}
	for (DepthBindings::RootVec::const_iterator it = data_->roots.begin(), end = data_->roots.end(); it != end; ++it) {
		nNodes_ = std::max(nNodes_, it->node + 1);
	}

	// Initialize node data
	nodes_.assign(nNodes_, DepthLitVec());
	maxVal_.assign(nNodes_, 0);
	for (DepthBindings::BindingVec::const_iterator it = data_->bindings.begin(), end = data_->bindings.end(); it != end; ++it) {
		uint32 v = it->depth > 0 ? static_cast<uint32>(it->depth) : 0u;
		DepthLit c = { it->depth, it->lit };
		nodes_[it->node].push_back(c);
		if (v > maxVal_[it->node]) { maxVal_[it->node] = v; }
	}

	// Determine root nodes: user-declared #root nodes when present,
	// otherwise static topology (nodes with no incoming edges)
	useDeclared_ = !data_->roots.empty();
	roots_.clear();
	if (!useDeclared_) {
		for (uint32 n = 0; n != nNodes_; ++n) {
			bool root = !graph_->validNode(n) || graph_->invBegin(n) == 0;
			if (root) { roots_.push_back(n); }
		}
	}

	// Initialize distance and BFS tracking
	distLo_.resize(nNodes_);
	distHi_.resize(nNodes_);
	parent_.resize(nNodes_);
	parentLit_.assign(nNodes_, lit_true());

	// Initialize reason construction
	mark_.assign(s.numVars() + 1, 0);
	epoch_ = 0;

	// Build edge literal -> edge index map for watch callbacks
	edgeToIdx_.assign(s.numVars() + 1, UINT32_MAX);
	for (uint32 i = 0, end = graph_->edges(); i != end; ++i) {
		const ExtDepGraph::Arc& a = graph_->arc(i);
		edgeToIdx_[a.lit.var()] = i;
	}

	// Initialize edge tracking
	assignedEdgeLits_.clear();
	lastTrailPos_ = 0;
	edgesDirty_ = false;

	// Initialize dirty node tracking
	prevDistLo_.assign(nNodes_, INF);
	prevDistHi_.assign(nNodes_, INF);
	dirtyNodes_.clear();

	// Watch all edge literals to detect assignments
	for (uint32 i = 0, end = graph_->edges(); i != end; ++i) {
		const ExtDepGraph::Arc& a = graph_->arc(i);
		s.addWatch(a.lit, this);
		s.addWatch(~a.lit, this);
	}

	return true;
}

void DepthPropagator::reset() {}

void DepthPropagator::destroy(Solver* s, bool detach) {
	if (s && detach) { s->removePost(this); }
	PostPropagator::destroy(s, detach);
}

void DepthPropagator::computeDist(Solver& s, U32Vec& dist, bool trueOnly) {
	auto start = std::chrono::high_resolution_clock::now();
	stats_.computeDistCalls++;

	// BFS: trueOnly=true uses only true edges, trueOnly=false uses non-false edges
	dist.assign(nNodes_, INF);
	U32Vec q;
	q.reserve(nNodes_);

	// Seed BFS from root nodes
	if (useDeclared_) {
		// User-declared #root nodes (only if condition literal is not false)
		for (DepthBindings::RootVec::const_iterator it = data_->roots.begin(), end = data_->roots.end(); it != end; ++it) {
			uint32 r = it->node;
			if (r >= nNodes_ || dist[r] == 0 || s.isFalse(it->lit)) { continue; }
			dist[r] = 0;
			parent_[r] = r;
			parentLit_[r] = lit_true();
			q.push_back(r);
		}
	} else {
		// Static topology roots (nodes with no incoming edges)
		for (U32Vec::const_iterator it = roots_.begin(), end = roots_.end(); it != end; ++it) {
			uint32 r = *it;
			dist[r] = 0;
			parent_[r] = r;
			parentLit_[r] = lit_true();
			q.push_back(r);
		}
	}

	// BFS traversal
	for (std::size_t h = 0; h != q.size(); ++h) {
		uint32 u = q[h];
		if (!graph_->validNode(u)) { continue; }
		uint32 du = dist[u];
		for (const ExtDepGraph::Arc* a = graph_->fwdBegin(u); a; a = graph_->fwdNext(a)) {
			bool usable = trueOnly ? s.isTrue(a->lit) : !s.isFalse(a->lit);
			if (!usable) { continue; }
			uint32 v = a->head();
			if (v < nNodes_ && dist[v] == INF) {
				dist[v] = du + 1;
				parent_[v] = u;
				parentLit_[v] = a->lit;
				q.push_back(v);
			}
		}
	}

	auto end = std::chrono::high_resolution_clock::now();
	stats_.totalBfsTimeMs += std::chrono::duration<double, std::milli>(end - start).count();
}

void DepthPropagator::updateAssignedEdgesFromTrail(Solver& s) {
	uint32 trailSize = s.numAssignedVars();

	// Check if we backtracked (trail shrunk)
	if (lastTrailPos_ > trailSize) {
		// Rebuild edge list from scratch after backtrack
		assignedEdgeLits_.clear();
		lastTrailPos_ = 0;
	}

	// Process new assignments incrementally
	for (uint32 i = lastTrailPos_; i < trailSize; ++i) {
		Literal lit = s.trail()[i];
		Var v = lit.var();
		// Check if this variable is an edge literal
		if (v < edgeToIdx_.size() && edgeToIdx_[v] != UINT32_MAX) {
			uint32 edgeIdx = edgeToIdx_[v];
			const ExtDepGraph::Arc& a = graph_->arc(edgeIdx);
			// Add the edge literal in its assigned polarity
			if (s.isTrue(a.lit)) {
				assignedEdgeLits_.push_back(a.lit);
			} else if (s.isFalse(a.lit)) {
				assignedEdgeLits_.push_back(~a.lit);
			}
		}
	}
	lastTrailPos_ = trailSize;
}

void DepthPropagator::setReason(Literal p, const LitVec& reason) {
	if (!nogoods_) { nogoods_ = new ReasonStore(); }
	nogoods_->setReason(p, reason);
}

bool DepthPropagator::propagateDepth(Solver& s) {
	if (!graph_) { return true; }
	auto startDepth = std::chrono::high_resolution_clock::now();
	stats_.propagateDepthCalls++;

	// Early exit if no edges changed since last propagation
	if (!edgesDirty_) {
		auto endDepth = std::chrono::high_resolution_clock::now();
		stats_.totalPropagateDepthTimeMs += std::chrono::duration<double, std::milli>(endDepth - startDepth).count();
		return true;
	}
	edgesDirty_ = false;

	// Update assigned edge cache incrementally from solver trail
	auto startEdgeCheck = std::chrono::high_resolution_clock::now();
	updateAssignedEdgesFromTrail(s);
	bool allEdgesAssigned = (assignedEdgeLits_.size() == graph_->edges());

	// Compute both distances: distLo (non-false edges) and distHi (true edges only)
	computeDist(s, distLo_, false);  // Optimistic: includes unassigned edges
	computeDist(s, distHi_, true);   // Pessimistic: only true edges

	// Identify nodes whose distance changed (dirty nodes)
	dirtyNodes_.clear();
	for (uint32 n = 0; n < nNodes_; ++n) {
		if (distLo_[n] != prevDistLo_[n] || distHi_[n] != prevDistHi_[n]) {
			dirtyNodes_.push_back(n);
			prevDistLo_[n] = distLo_[n];
			prevDistHi_[n] = distHi_[n];
		}
	}

	// Pre-build the all-edges reason once (reused for all forces in this round)
	cachedEdgeReason_.clear();
	cachedEdgeReason_.reserve(assignedEdgeLits_.size());
	for (LitVec::const_iterator it = assignedEdgeLits_.begin(), end = assignedEdgeLits_.end(); it != end; ++it) {
		cachedEdgeReason_.push_back(*it);
	}

	auto endEdgeCheck = std::chrono::high_resolution_clock::now();
	stats_.totalEdgeCheckTimeMs += std::chrono::duration<double, std::milli>(endEdgeCheck - startEdgeCheck).count();

	// Check constraints using two-distance propagation
	auto startConstraint = std::chrono::high_resolution_clock::now();

	// Check dirty nodes (or all nodes if edges complete)
	U32Vec nodesToCheck;
	if (allEdgesAssigned) {
		// Check all nodes when we have a complete assignment
		nodesToCheck.reserve(nNodes_);
		for (uint32 n = 0; n < nNodes_; ++n) {
			nodesToCheck.push_back(n);
		}
	} else {
		// Only check dirty nodes during partial assignment
		nodesToCheck = dirtyNodes_;
	}

	// Check depth constraints for each node
	for (U32Vec::const_iterator nit = nodesToCheck.begin(), nend = nodesToCheck.end(); nit != nend; ++nit) {
		uint32 n = *nit;
		DepthLitVec& cs = nodes_[n];
		if (cs.empty()) { continue; }

		uint32 mx = maxVal_[n];
		uint32 dLo = distLo_[n];
		uint32 dHi = distHi_[n];
		bool reachable = (dHi != INF);

		for (DepthLitVec::const_iterator it = cs.begin(), end = cs.end(); it != end; ++it) {
			stats_.totalLiteralsChecked++;

			int32 val = it->depth;
			Literal lit = it->lit;
			Literal force;

			if (allEdgesAssigned) {
				// All edges assigned: force exact value
				uint32 actualDepth = reachable ? dHi : mx;
				if (val == static_cast<int32>(actualDepth)) {
					force = lit;
					stats_.forcedCompletePositive++;
				} else {
					force = ~lit;
					stats_.forcedCompleteNegative++;
				}
			} else if (reachable && dLo == dHi) {
				// Depth is CERTAIN (locked in): force exact value even with partial assignment
				// This matches ASP #min aggregate behavior
				if (val == static_cast<int32>(dHi)) {
					force = lit;
					stats_.forcedCertainPositive++;
				} else {
					force = ~lit;
					stats_.forcedCertainNegative++;
				}
			} else {
				// Depth uncertain: only force bounds
				if (reachable && dLo != INF && val < static_cast<int32>(dLo)) {
					force = ~lit;  // Too small
					stats_.forcedBoundNegative++;
				} else if (reachable && dHi != INF && val > static_cast<int32>(dHi)) {
					force = ~lit;  // Too large
					stats_.forcedBoundNegative++;
				} else {
					continue;  // Can't determine yet
				}
			}

			if (s.isTrue(force)) { continue; }

			// Build reason clause using cached all-edges reason
			auto startReason = std::chrono::high_resolution_clock::now();
			setReason(force, cachedEdgeReason_);
			auto endReason = std::chrono::high_resolution_clock::now();
			stats_.totalReasonBuildTimeMs += std::chrono::duration<double, std::milli>(endReason - startReason).count();

			// Propagate the literal
			auto startForce = std::chrono::high_resolution_clock::now();
			bool forceOk = s.force(force, this);
			auto endForce = std::chrono::high_resolution_clock::now();
			stats_.totalForceTimeMs += std::chrono::duration<double, std::milli>(endForce - startForce).count();
			stats_.totalForceCalls++;

			if (!forceOk) {
				auto endConstraint = std::chrono::high_resolution_clock::now();
				stats_.totalConstraintCheckTimeMs += std::chrono::duration<double, std::milli>(endConstraint - startConstraint).count();
				auto endDepth = std::chrono::high_resolution_clock::now();
				stats_.totalPropagateDepthTimeMs += std::chrono::duration<double, std::milli>(endDepth - startDepth).count();
				return false;
			}
		}
	}
	auto endConstraint = std::chrono::high_resolution_clock::now();
	stats_.totalConstraintCheckTimeMs += std::chrono::duration<double, std::milli>(endConstraint - startConstraint).count();
	auto endDepth = std::chrono::high_resolution_clock::now();
	stats_.totalPropagateDepthTimeMs += std::chrono::duration<double, std::milli>(endDepth - startDepth).count();
	return true;
}

bool DepthPropagator::propagateFixpoint(Solver& s, PostPropagator*) {
	if (!graph_) { return true; }
	auto start = std::chrono::high_resolution_clock::now();
	stats_.propagateFixpointCalls++;

	uint32 before = s.numAssignedVars();
	if (!propagateDepth(s)) {
		auto end = std::chrono::high_resolution_clock::now();
		stats_.totalPropagateTimeMs += std::chrono::duration<double, std::milli>(end - start).count();
		return false;
	}
	if (s.numAssignedVars() == before) {
		auto end = std::chrono::high_resolution_clock::now();
		stats_.totalPropagateTimeMs += std::chrono::duration<double, std::milli>(end - start).count();
		return true;
	}

	auto startUntil = std::chrono::high_resolution_clock::now();
	bool result = s.propagateUntil(this);
	auto endUntil = std::chrono::high_resolution_clock::now();
	stats_.totalPropagateUntilTimeMs += std::chrono::duration<double, std::milli>(endUntil - startUntil).count();

	auto end = std::chrono::high_resolution_clock::now();
	stats_.totalPropagateTimeMs += std::chrono::duration<double, std::milli>(end - start).count();
	return result;
}

bool DepthPropagator::valid(Solver&) {
	return true;
}

bool DepthPropagator::isModel(Solver& s) {
	if (!graph_) { return true; }
	auto start = std::chrono::high_resolution_clock::now();
	bool result = propagateDepth(s);
	auto end = std::chrono::high_resolution_clock::now();
	stats_.totalPropagateTimeMs += std::chrono::duration<double, std::milli>(end - start).count();
	return result;
}

void DepthPropagator::reason(Solver&, Literal p, LitVec& out) {
	if (nogoods_) {
		nogoods_->getReason(p, out);
	}
}

Constraint::PropResult DepthPropagator::propagate(Solver&, Literal, uint32&) {
	// Watch callback: mark that edge assignments changed
	edgesDirty_ = true;
	return PropResult(true, true);  // ok=true, keepWatch=true
}

void DepthPropagator::printStats() const {
	if (stats_.propagateDepthCalls == 0) { return; }

	fprintf(stderr, "\n=== DepthPropagator Statistics ===\n");
	fprintf(stderr, "propagateDepth calls:    %llu\n",
	        (unsigned long long)stats_.propagateDepthCalls);
	fprintf(stderr, "computeDist (BFS) calls: %llu\n",
	        (unsigned long long)stats_.computeDistCalls);
	fprintf(stderr, "\n");
	fprintf(stderr, "Total propagate time:       %.3f ms\n", stats_.totalPropagateTimeMs);
	fprintf(stderr, "  propagateDepth time:      %.3f ms (%.1f%%)\n",
	        stats_.totalPropagateDepthTimeMs,
	        stats_.totalPropagateTimeMs > 0 ? 100.0 * stats_.totalPropagateDepthTimeMs / stats_.totalPropagateTimeMs : 0);
	fprintf(stderr, "    edge check time:        %.3f ms (%.1f%%)\n",
	        stats_.totalEdgeCheckTimeMs,
	        stats_.totalPropagateDepthTimeMs > 0 ? 100.0 * stats_.totalEdgeCheckTimeMs / stats_.totalPropagateDepthTimeMs : 0);
	fprintf(stderr, "    BFS time:               %.3f ms (%.1f%%)\n",
	        stats_.totalBfsTimeMs,
	        stats_.totalPropagateDepthTimeMs > 0 ? 100.0 * stats_.totalBfsTimeMs / stats_.totalPropagateDepthTimeMs : 0);
	fprintf(stderr, "    constraint check time:  %.3f ms (%.1f%%)\n",
	        stats_.totalConstraintCheckTimeMs,
	        stats_.totalPropagateDepthTimeMs > 0 ? 100.0 * stats_.totalConstraintCheckTimeMs / stats_.totalPropagateDepthTimeMs : 0);
	fprintf(stderr, "      literals checked:     %llu\n",
	        (unsigned long long)stats_.totalLiteralsChecked);
	fprintf(stderr, "      force calls:          %llu (%.1f avg per literal checked)\n",
	        (unsigned long long)stats_.totalForceCalls,
	        stats_.totalLiteralsChecked > 0 ? (double)stats_.totalForceCalls / stats_.totalLiteralsChecked : 0);
	fprintf(stderr, "      reason build time:    %.3f ms (%.1f%%) [%.3f us/reason]\n",
	        stats_.totalReasonBuildTimeMs,
	        stats_.totalConstraintCheckTimeMs > 0 ? 100.0 * stats_.totalReasonBuildTimeMs / stats_.totalConstraintCheckTimeMs : 0,
	        stats_.totalForceCalls > 0 ? stats_.totalReasonBuildTimeMs * 1000.0 / stats_.totalForceCalls : 0);
	fprintf(stderr, "      s.force() time:       %.3f ms (%.1f%%) [%.3f us/call]\n",
	        stats_.totalForceTimeMs,
	        stats_.totalConstraintCheckTimeMs > 0 ? 100.0 * stats_.totalForceTimeMs / stats_.totalConstraintCheckTimeMs : 0,
	        stats_.totalForceCalls > 0 ? stats_.totalForceTimeMs * 1000.0 / stats_.totalForceCalls : 0);
	double loopOverhead = stats_.totalConstraintCheckTimeMs - stats_.totalReasonBuildTimeMs - stats_.totalForceTimeMs;
	fprintf(stderr, "      loop overhead:        %.3f ms (%.1f%%)\n",
	        loopOverhead,
	        stats_.totalConstraintCheckTimeMs > 0 ? 100.0 * loopOverhead / stats_.totalConstraintCheckTimeMs : 0);
	fprintf(stderr, "  propagateUntil time:      %.3f ms (%.1f%%)\n",
	        stats_.totalPropagateUntilTimeMs,
	        stats_.totalPropagateTimeMs > 0 ? 100.0 * stats_.totalPropagateUntilTimeMs / stats_.totalPropagateTimeMs : 0);
	fprintf(stderr, "\n");
	fprintf(stderr, "Force breakdown:\n");
	fprintf(stderr, "  Certain depth (dLo==dHi): %llu positive, %llu negative\n",
	        (unsigned long long)stats_.forcedCertainPositive,
	        (unsigned long long)stats_.forcedCertainNegative);
	fprintf(stderr, "  Bounds (partial):         %llu negative\n",
	        (unsigned long long)stats_.forcedBoundNegative);
	fprintf(stderr, "  Complete assignment:      %llu positive, %llu negative\n",
	        (unsigned long long)stats_.forcedCompletePositive,
	        (unsigned long long)stats_.forcedCompleteNegative);
	uint64_t totalPartial = stats_.forcedCertainPositive + stats_.forcedCertainNegative + stats_.forcedBoundNegative;
	uint64_t totalComplete = stats_.forcedCompletePositive + stats_.forcedCompleteNegative;
	fprintf(stderr, "  Total partial:            %llu (%.1f%% of all forces)\n",
	        (unsigned long long)totalPartial,
	        stats_.totalForceCalls > 0 ? 100.0 * totalPartial / stats_.totalForceCalls : 0);
	fprintf(stderr, "  Total complete:           %llu (%.1f%% of all forces)\n",
	        (unsigned long long)totalComplete,
	        stats_.totalForceCalls > 0 ? 100.0 * totalComplete / stats_.totalForceCalls : 0);
	fprintf(stderr, "==================================\n\n");
}

} //namespace Clasp