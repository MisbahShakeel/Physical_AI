"""
Security and Rate Limiting Implementation for Textbook RAG System
"""

from fastapi import FastAPI, HTTPException, Depends, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
from pydantic import BaseModel
from typing import Optional, Dict, Any
import json
import jwt
from datetime import datetime, timedelta
import uuid
import hashlib
from cryptography.fernet import Fernet
import redis.asyncio as redis
import time
from enum import Enum


# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)

# Security schemes
security = HTTPBearer()

class UserRole(str, Enum):
    """User roles for authorization"""
    ANONYMOUS = "anonymous"
    USER = "user"
    ADMIN = "admin"

class SecurityManager:
    """Main security manager handling authentication and authorization"""

    def __init__(self, secret_key: str, redis_url: str = "redis://localhost:6379"):
        self.secret_key = secret_key
        self.algorithm = "HS256"
        self.redis_client = None
        self.encryption_key = Fernet.generate_key()
        self.cipher_suite = Fernet(self.encryption_key)

    async def initialize(self):
        """Initialize async components"""
        self.redis_client = redis.from_url(self.secret_key)

    async def create_access_token(self, user_id: str, role: UserRole = UserRole.USER, expires_delta: Optional[timedelta] = None):
        """Create JWT access token"""
        if expires_delta:
            expire = datetime.utcnow() + expires_delta
        else:
            expire = datetime.utcnow() + timedelta(hours=1)  # 1 hour default

        to_encode = {
            "sub": user_id,
            "role": role.value,
            "exp": expire,
            "iat": datetime.utcnow(),
            "jti": str(uuid.uuid4())  # JWT ID for revocation
        }

        encoded_jwt = jwt.encode(to_encode, self.secret_key, algorithm=self.algorithm)
        return encoded_jwt

    async def verify_token(self, token: str) -> Dict[str, Any]:
        """Verify JWT token and return payload"""
        try:
            payload = jwt.decode(token, self.secret_key, algorithms=[self.algorithm])
            return payload
        except jwt.ExpiredSignatureError:
            raise HTTPException(status_code=401, detail="Token has expired")
        except jwt.JWTError:
            raise HTTPException(status_code=401, detail="Invalid token")

    async def get_current_user(self, credentials: HTTPAuthorizationCredentials = Depends(security)) -> Dict[str, Any]:
        """Get current user from token"""
        token_data = await self.verify_token(credentials.credentials)
        return token_data

    async def is_admin(self, current_user: Dict[str, Any] = Depends(get_current_user)):
        """Check if current user has admin role"""
        if current_user.get("role") != UserRole.ADMIN.value:
            raise HTTPException(status_code=403, detail="Admin access required")
        return current_user


class RateLimiter:
    """Advanced rate limiting with multiple strategies"""

    def __init__(self, redis_client):
        self.redis = redis_client
        self.default_limits = {
            "anonymous": {"requests": 10, "window": 60},  # 10 requests per minute
            "user": {"requests": 100, "window": 60},      # 100 requests per minute
            "admin": {"requests": 1000, "window": 60}     # 1000 requests per minute
        }

    async def check_rate_limit(self, identifier: str, user_role: str = "anonymous") -> bool:
        """Check if request exceeds rate limit"""
        limit_config = self.default_limits.get(user_role, self.default_limits["anonymous"])
        requests, window = limit_config["requests"], limit_config["window"]

        # Create key for this user and time window
        current_time = int(time.time())
        window_start = current_time - (current_time % window)
        key = f"rate_limit:{identifier}:{window_start}"

        # Increment counter
        current_count = await self.redis.incr(key)

        if current_count == 1:
            # Set expiration for the key
            await self.redis.expire(key, window)

        return current_count <= requests

    async def get_reset_time(self, identifier: str, user_role: str = "anonymous") -> int:
        """Get time when rate limit resets"""
        limit_config = self.default_limits.get(user_role, self.default_limits["anonymous"])
        window = limit_config["window"]

        current_time = int(time.time())
        window_start = current_time - (current_time % window)
        return window_start + window


class SessionManager:
    """Manage user sessions and security"""

    def __init__(self, redis_client, postgres_client):
        self.redis = redis_client
        self.postgres = postgres_client
        self.session_timeout = 3600  # 1 hour

    async def create_session(self, user_id: str, ip_address: str, user_agent: str) -> str:
        """Create a new secure session"""
        session_id = str(uuid.uuid4())

        session_data = {
            "user_id": user_id,
            "ip_address": ip_address,
            "user_agent": user_agent,
            "created_at": datetime.utcnow().isoformat(),
            "last_activity": datetime.utcnow().isoformat(),
            "active": True
        }

        # Store in Redis with expiration
        await self.redis.setex(
            f"session:{session_id}",
            self.session_timeout,
            json.dumps(session_data)
        )

        # Also store in PostgreSQL for persistence
        await self.postgres.execute(
            """
            INSERT INTO sessions (id, user_id, ip_address, user_agent, created_at, last_activity, active)
            VALUES ($1, $2, $3, $4, $5, $6, $7)
            """,
            session_id, user_id, ip_address, user_agent,
            datetime.utcnow(), datetime.utcnow(), True
        )

        return session_id

    async def validate_session(self, session_id: str, ip_address: str) -> bool:
        """Validate session and check IP match"""
        # Check Redis first (faster)
        session_data = await self.redis.get(f"session:{session_id}")
        if session_data:
            session_data = json.loads(session_data)
            if session_data["ip_address"] != ip_address:
                # IP mismatch - possible session hijacking
                await self.invalidate_session(session_id)
                return False
            # Update last activity in Redis
            session_data["last_activity"] = datetime.utcnow().isoformat()
            await self.redis.setex(
                f"session:{session_id}",
                self.session_timeout,
                json.dumps(session_data)
            )
            return session_data["active"]

        # Fall back to PostgreSQL
        session = await self.postgres.fetchrow(
            """
            SELECT user_id, ip_address, active, last_activity
            FROM sessions
            WHERE id = $1 AND active = true AND last_activity > $2
            """,
            session_id,
            datetime.utcnow() - timedelta(seconds=self.session_timeout)
        )

        if not session:
            return False

        if session["ip_address"] != ip_address:
            # IP mismatch
            await self.invalidate_session(session_id)
            return False

        # Update last activity in PostgreSQL
        await self.postgres.execute(
            "UPDATE sessions SET last_activity = $1 WHERE id = $2",
            datetime.utcnow(), session_id
        )

        return session["active"]

    async def invalidate_session(self, session_id: str):
        """Invalidate a session"""
        await self.redis.delete(f"session:{session_id}")
        await self.postgres.execute(
            "UPDATE sessions SET active = false WHERE id = $1",
            session_id
        )


class DataPrivacyManager:
    """Handle data privacy and protection"""

    def __init__(self, encryption_key: bytes):
        self.cipher_suite = Fernet(encryption_key)

    def encrypt_data(self, data: str) -> str:
        """Encrypt sensitive data"""
        encrypted_data = self.cipher_suite.encrypt(data.encode())
        return encrypted_data.decode()

    def decrypt_data(self, encrypted_data: str) -> str:
        """Decrypt sensitive data"""
        decrypted_data = self.cipher_suite.decrypt(encrypted_data.encode())
        return decrypted_data.decode()

    def hash_pii(self, pii_data: str) -> str:
        """Hash personally identifiable information"""
        return hashlib.sha256(pii_data.encode()).hexdigest()

    def pseudonymize_data(self, data: str, salt: str = "") -> str:
        """Pseudonymize data by replacing with pseudonyms"""
        # Create a consistent pseudonym for the same data
        combined = data + salt
        hash_obj = hashlib.sha256(combined.encode())
        return f"pseudo_{hash_obj.hexdigest()[:12]}"

    async def anonymize_user_data(self, user_id: str, postgres_client):
        """Anonymize all user data for privacy compliance"""
        # Hash email addresses
        await postgres_client.execute(
            """
            UPDATE users
            SET email = $1
            WHERE id = $2
            """,
            self.hash_pii(f"anonymized_{user_id}"),
            user_id
        )

        # Remove direct PII from queries
        await postgres_client.execute(
            """
            UPDATE queries
            SET user_id = $1
            WHERE user_id = $2
            """,
            "anonymized",
            user_id
        )


class AuditLogger:
    """Log security events and API usage"""

    def __init__(self, postgres_client):
        self.postgres = postgres_client

    async def log_request(self, request: Request, user_id: Optional[str], response_status: int):
        """Log API request for audit purposes"""
        await self.postgres.execute(
            """
            INSERT INTO audit_logs (
                user_id,
                endpoint,
                method,
                ip_address,
                user_agent,
                response_status,
                timestamp
            )
            VALUES ($1, $2, $3, $4, $5, $6, $7)
            """,
            user_id,
            request.url.path,
            request.method,
            request.client.host,
            request.headers.get("user-agent", ""),
            response_status,
            datetime.utcnow()
        )

    async def log_security_event(self, event_type: str, user_id: Optional[str], details: Dict[str, Any]):
        """Log security-related events"""
        await self.postgres.execute(
            """
            INSERT INTO security_logs (
                event_type,
                user_id,
                ip_address,
                details,
                timestamp
            )
            VALUES ($1, $2, $3, $4, $5)
            """,
            event_type,
            user_id,
            details.get("ip_address", ""),
            json.dumps(details),
            datetime.utcnow()
        )


class SecurityMiddleware:
    """Security middleware for FastAPI"""

    def __init__(self, security_manager: SecurityManager, rate_limiter: RateLimiter):
        self.security_manager = security_manager
        self.rate_limiter = rate_limiter

    async def __call__(self, request: Request, call_next):
        # Get user from token if available
        user_id = None
        user_role = "anonymous"

        auth_header = request.headers.get("authorization")
        if auth_header and auth_header.startswith("Bearer "):
            token = auth_header[7:]
            try:
                payload = await self.security_manager.verify_token(token)
                user_id = payload.get("sub")
                user_role = payload.get("role", "user")
            except:
                # Invalid token, treat as anonymous
                pass

        # Check rate limit
        identifier = user_id or request.client.host
        is_allowed = await self.rate_limiter.check_rate_limit(identifier, user_role)

        if not is_allowed:
            return JSONResponse(
                status_code=429,
                content={"detail": "Rate limit exceeded"}
            )

        # Add security headers
        response = await call_next(request)
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"
        response.headers["X-XSS-Protection"] = "1; mode=block"

        return response


class InputValidator:
    """Validate and sanitize user inputs"""

    @staticmethod
    def validate_query(query: str) -> str:
        """Validate and sanitize query input"""
        if not query or len(query.strip()) == 0:
            raise HTTPException(status_code=400, detail="Query cannot be empty")

        if len(query) > 2000:  # Max query length
            raise HTTPException(status_code=400, detail="Query too long")

        # Sanitize query (remove potential injection characters)
        # In a real implementation, you'd want more sophisticated sanitization
        sanitized_query = query.strip()

        return sanitized_query

    @staticmethod
    def validate_selected_text(selected_text: str) -> str:
        """Validate and sanitize selected text"""
        if selected_text and len(selected_text) > 5000:  # Max selected text length
            raise HTTPException(status_code=400, detail="Selected text too long")

        return selected_text.strip() if selected_text else None


# Apply security to FastAPI app
def setup_security(app: FastAPI, secret_key: str):
    """Setup security for FastAPI application"""

    # Initialize security components
    security_manager = SecurityManager(secret_key)
    rate_limiter = RateLimiter(app.state.redis_client if hasattr(app.state, 'redis_client') else None)

    # Setup rate limiting
    app.state.limiter = rate_limiter
    app.state.security_manager = security_manager

    # Add rate limit exception handler
    app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

    # Add security middleware
    security_middleware = SecurityMiddleware(security_manager, rate_limiter)
    app.middleware("http")(security_middleware.__call__)

    return security_manager, rate_limiter


# Security dependencies
async def get_current_user_security(
    security_manager: SecurityManager = Depends(lambda: app.state.security_manager),
    credentials: HTTPAuthorizationCredentials = Depends(security)
):
    """Dependency to get current authenticated user"""
    token_data = await security_manager.verify_token(credentials.credentials)
    return token_data


async def require_authentication(
    current_user: Dict[str, Any] = Depends(get_current_user_security)
):
    """Dependency to require authentication"""
    return current_user


async def require_admin(
    current_user: Dict[str, Any] = Depends(get_current_user_security)
):
    """Dependency to require admin role"""
    if current_user.get("role") != UserRole.ADMIN.value:
        raise HTTPException(status_code=403, detail="Admin access required")
    return current_user


# Protected endpoints examples
def add_security_endpoints(app: FastAPI):
    """Add security-related endpoints to the application"""

    @app.get("/security/whoami")
    async def whoami(current_user: Dict[str, Any] = Depends(require_authentication)):
        """Get information about current authenticated user"""
        return {
            "user_id": current_user.get("sub"),
            "role": current_user.get("role"),
            "token_issued_at": current_user.get("iat"),
            "token_expires_at": current_user.get("exp")
        }

    @app.get("/security/rate-limit-status")
    async def rate_limit_status(
        request: Request,
        current_user: Dict[str, Any] = Depends(require_authentication)
    ):
        """Get current rate limit status"""
        user_id = current_user.get("sub")
        user_role = current_user.get("role", "user")

        is_allowed = await app.state.limiter.check_rate_limit(user_id, user_role)
        reset_time = await app.state.limiter.get_reset_time(user_id, user_role)

        limit_config = app.state.limiter.default_limits.get(user_role,
                                                          app.state.limiter.default_limits["anonymous"])

        return {
            "is_allowed": is_allowed,
            "limit": limit_config["requests"],
            "window": limit_config["window"],
            "reset_time": reset_time
        }

    @app.post("/security/logout")
    async def logout(
        request: Request,
        current_user: Dict[str, Any] = Depends(require_authentication),
        session_manager: SessionManager = Depends(lambda: app.state.session_manager)
    ):
        """Logout current user"""
        auth_header = request.headers.get("authorization")
        if auth_header and auth_header.startswith("Bearer "):
            token = auth_header[7:]
            # In a real implementation, you'd add the token to a blacklist
            pass

        return {"status": "logged_out"}


# Database schema for security tables
SECURITY_DB_SCHEMA = """
-- Audit logs table
CREATE TABLE IF NOT EXISTS audit_logs (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id),
    endpoint VARCHAR(255) NOT NULL,
    method VARCHAR(10) NOT NULL,
    ip_address INET,
    user_agent TEXT,
    response_status INTEGER,
    timestamp TIMESTAMP DEFAULT NOW()
);

-- Security events log
CREATE TABLE IF NOT EXISTS security_logs (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    event_type VARCHAR(100) NOT NULL,
    user_id UUID REFERENCES users(id),
    ip_address INET NOT NULL,
    details JSONB,
    timestamp TIMESTAMP DEFAULT NOW()
);

-- API keys table (for service authentication)
CREATE TABLE IF NOT EXISTS api_keys (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) NOT NULL,
    key_hash VARCHAR(255) NOT NULL,
    name VARCHAR(255) NOT NULL,
    permissions JSONB DEFAULT '["read"]'::jsonb,
    created_at TIMESTAMP DEFAULT NOW(),
    expires_at TIMESTAMP,
    last_used TIMESTAMP
);

-- Session management
CREATE TABLE IF NOT EXISTS sessions (
    id UUID PRIMARY KEY,
    user_id UUID REFERENCES users(id),
    ip_address INET NOT NULL,
    user_agent TEXT,
    created_at TIMESTAMP DEFAULT NOW(),
    last_activity TIMESTAMP DEFAULT NOW(),
    active BOOLEAN DEFAULT TRUE
);

-- Failed login attempts for brute force protection
CREATE TABLE IF NOT EXISTS failed_login_attempts (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    identifier VARCHAR(255) NOT NULL,  -- email or username
    ip_address INET NOT NULL,
    timestamp TIMESTAMP DEFAULT NOW(),
    user_agent TEXT
);
"""


class BruteForceProtection:
    """Protect against brute force attacks"""

    def __init__(self, redis_client, max_attempts: int = 5, window_minutes: int = 15):
        self.redis = redis_client
        self.max_attempts = max_attempts
        self.window_seconds = window_minutes * 60

    async def record_failed_attempt(self, identifier: str, ip_address: str):
        """Record a failed login attempt"""
        # Record attempt by identifier
        identifier_key = f"failed_login:{identifier}"
        ip_key = f"failed_login_ip:{ip_address}"

        # Use Redis to track attempts
        current_time = int(time.time())
        window_start = current_time - self.window_seconds

        # Add attempt with timestamp
        await self.redis.zadd(identifier_key, {str(current_time): current_time})
        await self.redis.zadd(ip_key, {str(current_time): current_time})

        # Clean up old attempts
        await self.redis.zremrangebyscore(identifier_key, 0, window_start)
        await self.redis.zremrangebyscore(ip_key, 0, window_start)

        # Set expiration
        await self.redis.expire(identifier_key, self.window_seconds)
        await self.redis.expire(ip_key, self.window_seconds)

    async def is_blocked(self, identifier: str, ip_address: str) -> bool:
        """Check if identifier or IP is blocked"""
        identifier_key = f"failed_login:{identifier}"
        ip_key = f"failed_login_ip:{ip_address}"

        # Count attempts in the window
        current_time = int(time.time())
        window_start = current_time - self.window_seconds

        id_attempts = await self.redis.zcount(identifier_key, window_start, current_time)
        ip_attempts = await self.redis.zcount(ip_key, window_start, current_time)

        return id_attempts >= self.max_attempts or ip_attempts >= self.max_attempts * 2