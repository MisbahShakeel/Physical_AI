"""
Performance Benchmarks and Optimization for Textbook RAG System
"""
import time
import asyncio
import statistics
from typing import Dict, List, Tuple, Any, Optional
from dataclasses import dataclass
from enum import Enum
import psutil
import GPUtil
import numpy as np
from pydantic import BaseModel
from datetime import datetime
import logging

logger = logging.getLogger(__name__)


class BenchmarkType(str, Enum):
    """Types of performance benchmarks"""
    EMBEDDING_GENERATION = "embedding_generation"
    VECTOR_SEARCH = "vector_search"
    LLM_INFERENCE = "llm_inference"
    END_TO_END = "end_to_end"
    DATABASE = "database"
    MEMORY = "memory"
    THROUGHPUT = "throughput"


@dataclass
class BenchmarkResult:
    """Result of a single benchmark test"""
    benchmark_type: BenchmarkType
    operation: str
    duration_ms: float
    throughput: Optional[float] = None  # operations per second
    memory_used_mb: Optional[float] = None
    cpu_usage: Optional[float] = None
    timestamp: datetime = None

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.utcnow()


class PerformanceMonitor:
    """Monitor system performance during operations"""

    def __init__(self):
        self.benchmark_results: List[BenchmarkResult] = []

    def start_monitoring(self) -> Dict[str, Any]:
        """Capture initial system state"""
        return {
            "cpu_percent": psutil.cpu_percent(interval=None),
            "memory_percent": psutil.virtual_memory().percent,
            "memory_available_mb": psutil.virtual_memory().available / (1024 * 1024),
            "timestamp": time.time()
        }

    def end_monitoring(self, start_state: Dict[str, Any]) -> Dict[str, Any]:
        """Calculate performance metrics"""
        end_state = {
            "cpu_percent": psutil.cpu_percent(interval=None),
            "memory_percent": psutil.virtual_memory().percent,
            "memory_available_mb": psutil.virtual_memory().available / (1024 * 1024),
            "timestamp": time.time()
        }

        duration = end_state["timestamp"] - start_state["timestamp"]
        memory_used_mb = start_state["memory_available_mb"] - end_state["memory_available_mb"]

        return {
            "duration_s": duration,
            "memory_used_mb": abs(memory_used_mb),
            "cpu_avg": (start_state["cpu_percent"] + end_state["cpu_percent"]) / 2
        }


class BenchmarkSuite:
    """Comprehensive benchmark suite for the RAG system"""

    def __init__(self):
        self.monitor = PerformanceMonitor()
        self.target_metrics = self._define_target_metrics()

    def _define_target_metrics(self) -> Dict[BenchmarkType, Dict[str, float]]:
        """Define target performance metrics"""
        return {
            BenchmarkType.EMBEDDING_GENERATION: {
                "max_duration_ms": 500,  # Per chunk
                "target_throughput": 20,  # Chunks per second
                "max_memory_mb": 100
            },
            BenchmarkType.VECTOR_SEARCH: {
                "max_duration_ms": 200,  # Per search
                "target_throughput": 50,  # Searches per second
                "max_memory_mb": 50
            },
            BenchmarkType.LLM_INFERENCE: {
                "max_duration_ms": 1500,  # Per response
                "target_throughput": 5,   # Responses per second
                "max_memory_mb": 200
            },
            BenchmarkType.END_TO_END: {
                "max_duration_ms": 2500,  # Full query-to-response
                "target_throughput": 3,   # Queries per second
                "max_memory_mb": 300
            },
            BenchmarkType.THROUGHPUT: {
                "target_concurrent_users": 100,
                "max_response_time_95th": 2000,
                "success_rate": 0.95
            }
        }

    async def benchmark_embedding_generation(self, sample_texts: List[str]) -> BenchmarkResult:
        """Benchmark embedding generation performance"""
        start_state = self.monitor.start_monitoring()

        start_time = time.time()
        # Simulate embedding generation
        for text in sample_texts:
            # Simulate API call delay
            await asyncio.sleep(0.05)  # 50ms per embedding (simulated)
        end_time = time.time()

        perf_data = self.monitor.end_monitoring(start_state)
        duration_ms = (end_time - start_time) * 1000

        result = BenchmarkResult(
            benchmark_type=BenchmarkType.EMBEDDING_GENERATION,
            operation="generate_embeddings",
            duration_ms=duration_ms,
            throughput=len(sample_texts) / (duration_ms / 1000) if duration_ms > 0 else 0,
            memory_used_mb=perf_data["memory_used_mb"],
            cpu_usage=perf_data["cpu_avg"]
        )

        self.monitor.benchmark_results.append(result)
        return result

    async def benchmark_vector_search(self, query_embedding: List[float], num_searches: int = 100) -> BenchmarkResult:
        """Benchmark vector search performance"""
        start_state = self.monitor.start_monitoring()

        start_time = time.time()
        # Simulate vector search operations
        for _ in range(num_searches):
            # Simulate search delay
            await asyncio.sleep(0.002)  # 2ms per search (simulated)
        end_time = time.time()

        perf_data = self.monitor.end_monitoring(start_state)
        duration_ms = (end_time - start_time) * 1000

        result = BenchmarkResult(
            benchmark_type=BenchmarkType.VECTOR_SEARCH,
            operation="semantic_search",
            duration_ms=duration_ms,
            throughput=num_searches / (duration_ms / 1000) if duration_ms > 0 else 0,
            memory_used_mb=perf_data["memory_used_mb"],
            cpu_usage=perf_data["cpu_avg"]
        )

        self.monitor.benchmark_results.append(result)
        return result

    async def benchmark_llm_inference(self, prompts: List[str]) -> BenchmarkResult:
        """Benchmark LLM inference performance"""
        start_state = self.monitor.start_monitoring()

        start_time = time.time()
        # Simulate LLM inference
        for prompt in prompts:
            # Simulate API call delay
            await asyncio.sleep(0.3)  # 300ms per inference (simulated)
        end_time = time.time()

        perf_data = self.monitor.end_monitoring(start_state)
        duration_ms = (end_time - start_time) * 1000

        result = BenchmarkResult(
            benchmark_type=BenchmarkType.LLM_INFERENCE,
            operation="generate_response",
            duration_ms=duration_ms,
            throughput=len(prompts) / (duration_ms / 1000) if duration_ms > 0 else 0,
            memory_used_mb=perf_data["memory_used_mb"],
            cpu_usage=perf_data["cpu_avg"]
        )

        self.monitor.benchmark_results.append(result)
        return result

    async def benchmark_end_to_end(self, queries: List[str]) -> BenchmarkResult:
        """Benchmark complete end-to-end performance"""
        start_state = self.monitor.start_monitoring()

        start_time = time.time()
        # Simulate full query processing
        for query in queries:
            # Simulate full pipeline: retrieve + generate + verify
            await asyncio.sleep(0.8)  # 800ms per full query (simulated)
        end_time = time.time()

        perf_data = self.monitor.end_monitoring(start_state)
        duration_ms = (end_time - start_time) * 1000

        result = BenchmarkResult(
            benchmark_type=BenchmarkType.END_TO_END,
            operation="full_query_pipeline",
            duration_ms=duration_ms,
            throughput=len(queries) / (duration_ms / 1000) if duration_ms > 0 else 0,
            memory_used_mb=perf_data["memory_used_mb"],
            cpu_usage=perf_data["cpu_avg"]
        )

        self.monitor.benchmark_results.append(result)
        return result

    async def benchmark_throughput(self, concurrent_queries: int = 100) -> BenchmarkResult:
        """Benchmark system throughput under load"""
        start_state = self.monitor.start_monitoring()

        start_time = time.time()

        # Create concurrent tasks
        tasks = []
        for i in range(concurrent_queries):
            task = asyncio.create_task(self._simulate_query(i))
            tasks.append(task)

        # Wait for all tasks to complete
        results = await asyncio.gather(*tasks, return_exceptions=True)

        end_time = time.time()

        perf_data = self.monitor.end_monitoring(start_state)
        duration_ms = (end_time - start_time) * 1000

        successful_queries = sum(1 for r in results if not isinstance(r, Exception))

        result = BenchmarkResult(
            benchmark_type=BenchmarkType.THROUGHPUT,
            operation="concurrent_queries",
            duration_ms=duration_ms,
            throughput=successful_queries / (duration_ms / 1000) if duration_ms > 0 else 0,
            memory_used_mb=perf_data["memory_used_mb"],
            cpu_usage=perf_data["cpu_avg"]
        )

        self.monitor.benchmark_results.append(result)
        return result

    async def _simulate_query(self, query_id: int) -> str:
        """Simulate a single query operation"""
        # Simulate processing time
        await asyncio.sleep(0.05 + (query_id % 10) * 0.01)  # Variable processing time
        return f"Result for query {query_id}"

    def get_performance_report(self) -> Dict[str, Any]:
        """Generate comprehensive performance report"""
        if not self.monitor.benchmark_results:
            return {"error": "No benchmark results available"}

        report = {
            "timestamp": datetime.utcnow().isoformat(),
            "total_benchmarks": len(self.monitor.benchmark_results),
            "benchmarks_by_type": {},
            "overall_metrics": {},
            "compliance_status": {}
        }

        # Group results by type
        for result in self.monitor.benchmark_results:
            benchmark_type = result.benchmark_type.value
            if benchmark_type not in report["benchmarks_by_type"]:
                report["benchmarks_by_type"][benchmark_type] = []

            report["benchmarks_by_type"][benchmark_type].append({
                "operation": result.operation,
                "duration_ms": result.duration_ms,
                "throughput": result.throughput,
                "memory_used_mb": result.memory_used_mb,
                "cpu_usage": result.cpu_usage,
                "timestamp": result.timestamp.isoformat()
            })

        # Calculate aggregate metrics
        for benchmark_type, results in report["benchmarks_by_type"].items():
            durations = [r["duration_ms"] for r in results]
            throughputs = [r["throughput"] for r in results if r["throughput"] is not None]

            report["overall_metrics"][benchmark_type] = {
                "count": len(results),
                "avg_duration_ms": statistics.mean(durations) if durations else 0,
                "median_duration_ms": statistics.median(durations) if durations else 0,
                "p95_duration_ms": np.percentile(durations, 95) if durations else 0,
                "max_duration_ms": max(durations) if durations else 0,
                "min_duration_ms": min(durations) if durations else 0,
                "avg_throughput": statistics.mean(throughputs) if throughputs else 0,
                "total_memory_used_mb": sum(r["memory_used_mb"] or 0 for r in results)
            }

            # Check compliance with targets
            targets = self.target_metrics.get(BenchmarkType(benchmark_type), {})
            avg_duration = report["overall_metrics"][benchmark_type]["avg_duration_ms"]
            avg_throughput = report["overall_metrics"][benchmark_type]["avg_throughput"]

            report["compliance_status"][benchmark_type] = {
                "duration_compliant": avg_duration <= targets.get("max_duration_ms", float('inf')),
                "throughput_compliant": avg_throughput >= targets.get("target_throughput", 0),
                "memory_compliant": report["overall_metrics"][benchmark_type]["total_memory_used_mb"] <= targets.get("max_memory_mb", float('inf'))
            }

        return report

    def generate_optimization_recommendations(self) -> List[str]:
        """Generate optimization recommendations based on benchmark results"""
        recommendations = []

        if not self.monitor.benchmark_results:
            return ["Run benchmarks first to generate recommendations"]

        # Analyze results and provide recommendations
        for benchmark_type, results in self._group_results_by_type().items():
            avg_duration = statistics.mean([r.duration_ms for r in results])
            avg_throughput = statistics.mean([r.throughput for r in results if r.throughput is not None])

            targets = self.target_metrics.get(benchmark_type, {})
            max_duration = targets.get("max_duration_ms", float('inf'))
            target_throughput = targets.get("target_throughput", 0)

            if avg_duration > max_duration * 0.8:  # If within 20% of target
                recommendations.append(
                    f"{benchmark_type.value}: Duration ({avg_duration:.2f}ms) approaching target ({max_duration}ms). "
                    f"Consider optimization through caching or async processing."
                )

            if avg_throughput < target_throughput * 0.8:  # If below 80% of target
                recommendations.append(
                    f"{benchmark_type.value}: Throughput ({avg_throughput:.2f}/s) below target ({target_throughput}/s). "
                    f"Consider scaling resources or optimizing the processing pipeline."
                )

        # Add general recommendations
        if len(self.monitor.benchmark_results) > 100:  # If we have substantial data
            avg_memory = statistics.mean([
                r.memory_used_mb for r in self.monitor.benchmark_results
                if r.memory_used_mb is not None
            ])
            if avg_memory > 200:  # High memory usage
                recommendations.append(
                    "High memory usage detected. Consider implementing memory pooling or more efficient data structures."
                )

        return recommendations

    def _group_results_by_type(self) -> Dict[BenchmarkType, List[BenchmarkResult]]:
        """Group benchmark results by type"""
        grouped = {}
        for result in self.monitor.benchmark_results:
            if result.benchmark_type not in grouped:
                grouped[result.benchmark_type] = []
            grouped[result.benchmark_type].append(result)
        return grouped


class QdrantOptimizer:
    """Optimization strategies for Qdrant vector database"""

    @staticmethod
    def get_optimized_collection_config() -> Dict[str, Any]:
        """Get optimized Qdrant collection configuration"""
        return {
            "vectors": {
                "size": 1536,  # OpenAI embedding dimension
                "distance": "Cosine"
            },
            "hnsw_config": {
                "m": 16,  # Number of edges per node
                "ef_construct": 100,  # Construction time parameter
                "ef": 50,  # Query time parameter
                "max_indexing_threads": 0  # Use all available threads
            },
            "optimizer_config": {
                "deleted_threshold": 0.2,
                "vacuum_min_vector_number": 1000,
                "max_segment_number": 5,
                "memmap_threshold": 50000,
                "indexing_threshold": 20000
            },
            "wal_config": {
                "wal_capacity_mb": 32,
                "wal_segments_ahead": 0
            }
        }

    @staticmethod
    def get_optimized_search_params() -> Dict[str, Any]:
        """Get optimized search parameters for better performance"""
        return {
            "with_payload": True,
            "with_vectors": False,  # Don't return vectors unless needed
            "score_threshold": 0.3,  # Filter out low-relevance results
            "limit": 10  # Limit results to reduce processing time
        }


class CacheOptimizer:
    """Optimization strategies for caching"""

    def __init__(self, redis_client):
        self.redis = redis_client

    async def setup_optimized_cache(self):
        """Setup optimized cache configuration"""
        # Configure Redis for optimal performance
        await self.redis.config_set("maxmemory-policy", "allkeys-lru")
        await self.redis.config_set("hash-max-ziplist-entries", 512)
        await self.redis.config_set("hash-max-ziplist-value", 64)
        await self.redis.config_set("list-max-ziplist-size", -2)

    async def get_cache_hit_rate(self) -> float:
        """Calculate cache hit rate"""
        info = await self.redis.info("stats")
        hits = info.get("keyspace_hits", 0)
        misses = info.get("keyspace_misses", 1)  # Avoid division by zero
        total = hits + misses
        return hits / total if total > 0 else 0


class DatabaseOptimizer:
    """Optimization strategies for PostgreSQL"""

    @staticmethod
    def get_optimized_queries() -> Dict[str, str]:
        """Get optimized SQL queries with proper indexing"""
        return {
            "query_chunks_by_document": """
                SELECT id, content, section_title, chapter_title, token_count
                FROM chunks
                WHERE document_id = $1
                ORDER BY chunk_index
                LIMIT $2
            """,
            "search_chunks_by_content": """
                SELECT id, content, section_title, chapter_title
                FROM chunks
                WHERE to_tsvector('english', content) @@ plainto_tsquery('english', $1)
                ORDER BY ts_rank_cd(to_tsvector('english', content), plainto_tsquery('english', $1)) DESC
                LIMIT $2
            """,
            "get_recent_queries": """
                SELECT query_text, response_text, latency_ms, created_at
                FROM queries
                WHERE session_id = $1 AND created_at > $2
                ORDER BY created_at DESC
                LIMIT $3
            """
        }

    @staticmethod
    def get_suggested_indexes() -> List[str]:
        """Get suggested database indexes for optimal performance"""
        return [
            "CREATE INDEX IF NOT EXISTS idx_chunks_document_id ON chunks(document_id);",
            "CREATE INDEX IF NOT EXISTS idx_chunks_section_title ON chunks(section_title);",
            "CREATE INDEX IF NOT EXISTS idx_queries_session_id ON queries(session_id);",
            "CREATE INDEX IF NOT EXISTS idx_queries_created_at ON queries(created_at);",
            "CREATE INDEX IF NOT EXISTS idx_sessions_last_activity ON sessions(last_activity);",
            # Full-text search index
            "CREATE INDEX IF NOT EXISTS idx_chunks_content_gin ON chunks USING gin(to_tsvector('english', content));"
        ]


class LoadTester:
    """Load testing for the RAG system"""

    def __init__(self, api_client):
        self.api_client = api_client

    async def run_load_test(
        self,
        num_users: int,
        queries_per_user: int,
        duration_seconds: int = 60
    ) -> Dict[str, Any]:
        """Run load test with specified parameters"""
        start_time = time.time()
        end_time = start_time + duration_seconds

        results = []
        errors = []

        async def user_session(user_id: int):
            user_results = []
            for i in range(queries_per_user):
                if time.time() > end_time:
                    break

                query_start = time.time()
                try:
                    # Simulate API call
                    response = await self._make_query(f"Test query {user_id}-{i}")
                    query_time = (time.time() - query_start) * 1000  # Convert to ms

                    user_results.append({
                        "user_id": user_id,
                        "query_id": i,
                        "response_time_ms": query_time,
                        "success": True,
                        "status_code": 200
                    })
                except Exception as e:
                    query_time = (time.time() - query_start) * 1000
                    user_results.append({
                        "user_id": user_id,
                        "query_id": i,
                        "response_time_ms": query_time,
                        "success": False,
                        "error": str(e)
                    })
                    errors.append(str(e))

                # Small delay between queries
                await asyncio.sleep(0.1)

            return user_results

        # Create and run user tasks
        tasks = [user_session(user_id) for user_id in range(num_users)]
        all_results = await asyncio.gather(*tasks)

        # Flatten results
        flat_results = [item for sublist in all_results for item in sublist]

        actual_duration = time.time() - start_time
        successful_requests = [r for r in flat_results if r["success"]]
        failed_requests = [r for r in flat_results if not r["success"]]

        response_times = [r["response_time_ms"] for r in successful_requests]

        return {
            "test_config": {
                "num_users": num_users,
                "queries_per_user": queries_per_user,
                "duration_seconds": duration_seconds,
                "total_queries": num_users * queries_per_user
            },
            "results": {
                "actual_duration_seconds": actual_duration,
                "successful_requests": len(successful_requests),
                "failed_requests": len(failed_requests),
                "success_rate": len(successful_requests) / len(flat_results) if flat_results else 0,
                "throughput_per_second": len(successful_requests) / actual_duration if actual_duration > 0 else 0,
                "avg_response_time_ms": statistics.mean(response_times) if response_times else 0,
                "median_response_time_ms": statistics.median(response_times) if response_times else 0,
                "p95_response_time_ms": np.percentile(response_times, 95) if response_times else 0,
                "max_response_time_ms": max(response_times) if response_times else 0,
                "min_response_time_ms": min(response_times) if response_times else 0
            },
            "errors": errors
        }

    async def _make_query(self, query_text: str):
        """Make a query to the API"""
        # This would make actual API calls in a real implementation
        # For simulation, we'll just wait for a bit
        await asyncio.sleep(0.1)
        return {"response": f"Response to: {query_text}", "citations": []}


# Performance monitoring middleware
class PerformanceMonitoringMiddleware:
    """Middleware to monitor API performance in real-time"""

    def __init__(self, benchmark_suite: BenchmarkSuite):
        self.benchmark_suite = benchmark_suite

    async def __call__(self, request, call_next):
        start_time = time.time()
        start_state = self.benchmark_suite.monitor.start_monitoring()

        response = await call_next(request)

        end_time = time.time()
        perf_data = self.benchmark_suite.monitor.end_monitoring(start_state)
        duration_ms = (end_time - start_time) * 1000

        # Log performance for this request
        result = BenchmarkResult(
            benchmark_type=BenchmarkType.END_TO_END,
            operation=f"api_call_{request.method}_{request.url.path}",
            duration_ms=duration_ms,
            memory_used_mb=perf_data["memory_used_mb"],
            cpu_usage=perf_data["cpu_avg"]
        )

        self.benchmark_suite.monitor.benchmark_results.append(result)

        # Add performance headers to response
        response.headers["X-Response-Time"] = f"{duration_ms:.2f}ms"
        response.headers["X-Server-Time"] = datetime.utcnow().isoformat()

        return response


def setup_performance_monitoring(app):
    """Setup performance monitoring for the FastAPI app"""
    benchmark_suite = BenchmarkSuite()
    middleware = PerformanceMonitoringMiddleware(benchmark_suite)

    # Add middleware to app
    app.middleware("http")(middleware.__call__)

    # Store benchmark suite in app state
    app.state.benchmark_suite = benchmark_suite

    return benchmark_suite


# Performance API endpoints
def add_performance_endpoints(app):
    """Add performance monitoring endpoints to the application"""

    @app.get("/performance/report")
    async def get_performance_report():
        """Get comprehensive performance report"""
        if not hasattr(app.state, 'benchmark_suite'):
            return {"error": "Performance monitoring not initialized"}

        return app.state.benchmark_suite.get_performance_report()

    @app.get("/performance/recommendations")
    async def get_optimization_recommendations():
        """Get optimization recommendations"""
        if not hasattr(app.state, 'benchmark_suite'):
            return {"error": "Performance monitoring not initialized"}

        return {
            "recommendations": app.state.benchmark_suite.generate_optimization_recommendations(),
            "timestamp": datetime.utcnow().isoformat()
        }

    @app.post("/performance/run-benchmarks")
    async def run_comprehensive_benchmarks():
        """Run comprehensive benchmark tests"""
        if not hasattr(app.state, 'benchmark_suite'):
            return {"error": "Performance monitoring not initialized"}

        benchmark_suite = app.state.benchmark_suite

        # Run sample benchmarks
        await benchmark_suite.benchmark_embedding_generation(["test"] * 10)
        await benchmark_suite.benchmark_vector_search([0.1] * 1536, 50)
        await benchmark_suite.benchmark_llm_inference(["prompt"] * 5)
        await benchmark_suite.benchmark_end_to_end(["query"] * 3)
        await benchmark_suite.benchmark_throughput(20)

        return {
            "status": "benchmarks_completed",
            "total_results": len(benchmark_suite.monitor.benchmark_results),
            "timestamp": datetime.utcnow().isoformat()
        }

    @app.get("/performance/realtime-metrics")
    async def get_realtime_metrics():
        """Get real-time performance metrics"""
        return {
            "cpu_percent": psutil.cpu_percent(interval=1),
            "memory_percent": psutil.virtual_memory().percent,
            "memory_available_mb": psutil.virtual_memory().available / (1024 * 1024),
            "disk_usage_percent": psutil.disk_usage('/').percent,
            "timestamp": datetime.utcnow().isoformat()
        }