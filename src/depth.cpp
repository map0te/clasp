#include <clasp/depth.h>
#include <clasp/solver.h>
#include <clasp/shared_context.h>
#include <clasp/dependency_graph.h>
#include <algorithm>

namespace Clasp {

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

DepthPropagator::DepthPropagator(ExtDepGraph* graph, DepthBindings* data) : graph_(graph), data_(data), nogoods_(0), epoch_(0), nNodes_(0), useDeclared_(false) {}

DepthPropagator::~DepthPropagator() { delete nogoods_; }

bool DepthPropagator::init(Solver& s) {
	if (!graph_ || !data_ || data_->empty()) { graph_ = 0; return true; }
	nNodes_ = graph_->nodes();
	for (DepthBindings::BindingVec::const_iterator it = data_->bindings.begin(), end = data_->bindings.end(); it != end; ++it) {
		nNodes_ = std::max(nNodes_, it->node + 1);
	}
	for (DepthBindings::RootVec::const_iterator it = data_->roots.begin(), end = data_->roots.end(); it != end; ++it) {
		nNodes_ = std::max(nNodes_, it->node + 1);
	}
	nodes_.assign(nNodes_, DepthLitVec());
	maxVal_.assign(nNodes_, 0);
	for (DepthBindings::BindingVec::const_iterator it = data_->bindings.begin(), end = data_->bindings.end(); it != end; ++it) {
		uint32 v = it->depth > 0 ? static_cast<uint32>(it->depth) : 0u;
		DepthLit c = { it->depth, it->lit };
		nodes_[it->node].push_back(c);
		if (v > maxVal_[it->node]) { maxVal_[it->node] = v; }
	}
	// Seeds: user-declared #root nodes when present, otherwise fall back to static
	// topology (nodes with no incoming arc in the #edge graph).
	useDeclared_ = !data_->roots.empty();
	roots_.clear();
	if (!useDeclared_) {
		for (uint32 n = 0; n != nNodes_; ++n) {
			bool root = !graph_->validNode(n) || graph_->invBegin(n) == 0;
			if (root) { roots_.push_back(n); }
		}
	}
	distLo_.resize(nNodes_);
	distHi_.resize(nNodes_);
	parent_.resize(nNodes_);
	parentLit_.assign(nNodes_, lit_true());
	mark_.assign(s.numVars() + 1, 0);
	epoch_ = 0;
	return true;
}

void DepthPropagator::reset() {}

void DepthPropagator::destroy(Solver* s, bool detach) {
	if (s && detach) { s->removePost(this); }
	PostPropagator::destroy(s, detach);
}

void DepthPropagator::computeDist(Solver& s, U32Vec& dist, bool trueOnly, bool withParents) {
	dist.assign(nNodes_, INF);
	U32Vec q;
	q.reserve(nNodes_);
	if (useDeclared_) {
		// Seed from user-declared #root nodes whose condition literal is not false.
		for (DepthBindings::RootVec::const_iterator it = data_->roots.begin(), end = data_->roots.end(); it != end; ++it) {
			uint32 r = it->node;
			if (r >= nNodes_ || dist[r] == 0 || s.isFalse(it->lit)) { continue; }
			dist[r] = 0;
			if (withParents) { parent_[r] = r; parentLit_[r] = lit_true(); }
			q.push_back(r);
		}
	}
	else {
		for (U32Vec::const_iterator it = roots_.begin(), end = roots_.end(); it != end; ++it) {
			uint32 r = *it;
			dist[r] = 0;
			if (withParents) { parent_[r] = r; parentLit_[r] = lit_true(); }
			q.push_back(r);
		}
	}
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
				if (withParents) { parent_[v] = u; parentLit_[v] = a->lit; }
				q.push_back(v);
			}
		}
	}
}

void DepthPropagator::buildLowerReason(Solver& s, LitVec& out) {
	for (uint32 i = 0, end = graph_->edges(); i != end; ++i) {
		const ExtDepGraph::Arc& a = graph_->arc(i);
		if (s.isFalse(a.lit)) {
			Literal t = ~a.lit; 
			Var v = t.var();
			if (v >= mark_.size()) { mark_.resize(v + 1, 0); }
			if (mark_[v] != epoch_) { mark_[v] = epoch_; out.push_back(t); }
		}
	}
}

void DepthPropagator::appendTruePath(Solver& s, uint32 node, LitVec& out) {
	uint32 n = node;
	while (graph_->validNode(n) && parent_[n] != n) { 
		Literal e = parentLit_[n];
		Var v = e.var();
		if (v >= mark_.size()) { mark_.resize(v + 1, 0); }
		if (mark_[v] != epoch_) { mark_[v] = epoch_; out.push_back(e); }
		n = parent_[n];
	}
	static_cast<void>(s);
}

void DepthPropagator::setReason(Literal p, const LitVec& reason) {
	if (!nogoods_) { nogoods_ = new ReasonStore(); }
	nogoods_->setReason(p, reason);
}

bool DepthPropagator::propagateDepth(Solver& s) {
	if (!graph_) { return true; }
	computeDist(s, distLo_, false, false);
	computeDist(s, distHi_, true,  true);
	for (uint32 n = 0; n != nNodes_; ++n) {
		DepthLitVec& cs = nodes_[n];
		if (cs.empty()) { continue; } 
		uint32 mx = maxVal_[n];
		uint32 lo = distLo_[n] == INF ? mx : std::min(distLo_[n], mx);
		uint32 hi = distHi_[n] == INF ? mx : std::min(distHi_[n], mx);
		bool reachable = distHi_[n] != INF; 
		for (DepthLitVec::const_iterator it = cs.begin(), end = cs.end(); it != end; ++it) {
			int32 val = it->depth;
			Literal lit = it->lit;
			bool below = val < static_cast<int32>(lo); 
			bool above = val > static_cast<int32>(hi); 
			Literal force;                 
			bool needLower = false, needPath = false; 
			if (below || above) {
				force = ~lit;          
				needLower = below;     
				needPath = above;      
			}
			else if (lo == hi) {
				force = lit;           
				needLower = true;      
				needPath = reachable;  
			}
			else {
				continue;              
			}
			if (s.isTrue(force)) { continue; } 
			if (++epoch_ == 0) { mark_.assign(mark_.size(), 0); epoch_ = 1; }
			reason_.clear();
			if (needLower) { buildLowerReason(s, reason_); }
			if (needPath)  { appendTruePath(s, n, reason_); }
			setReason(force, reason_); 
			if (!s.force(force, this)) { return false; } 
		}
	}
	return true;
}

bool DepthPropagator::propagateFixpoint(Solver& s, PostPropagator*) {
	if (!graph_) { return true; }
	uint32 before = s.numAssignedVars();
	if (!propagateDepth(s)) { return false; }
	if (s.numAssignedVars() == before) { return true; }
	return s.propagateUntil(this); 
}

bool DepthPropagator::valid(Solver& s) {
	static_cast<void>(s);
	return true;
}

bool DepthPropagator::isModel(Solver& s) {
	if (!graph_) { return true; }
	return propagateDepth(s);
}

void DepthPropagator::reason(Solver&, Literal p, LitVec& out) {
	if (nogoods_) { nogoods_->getReason(p, out); }
}
} //namespace Clasp