"""Tests for RBAC (Role-Based Access Control) policy checking.

Tests the access_enforcer logic for operator, engineer, and viewer roles
covering topic/service access, wildcard rules, denied topics, default
policies, and unknown roles.
"""

import pytest


class _AccessPolicy:
    """Simplified AccessPolicy for testing."""
    def __init__(self, role, allowed_topics=None, allowed_services=None, denied_topics=None):
        self.role = role
        self.allowed_topics = allowed_topics or set()
        self.allowed_services = allowed_services or set()
        self.denied_topics = denied_topics or set()


class _AccessEnforcer:
    """Extracted access-check logic from AccessEnforcerNode.

    Replicates the RBAC policy enforcement without ROS2 dependencies.
    """

    def __init__(self, default_policy='deny'):
        self.default_policy = default_policy
        self.policies = {}
        self.violation_count = 0

    def add_policy(self, policy: _AccessPolicy):
        self.policies[policy.role] = policy

    def check_access(self, role: str, resource: str, resource_type: str = 'topic') -> bool:
        """Check if a role has access to a resource."""
        policy = self.policies.get(role)
        if policy is None:
            if self.default_policy == 'deny':
                self.violation_count += 1
                return False
            return True

        if resource_type == 'topic':
            allowed = policy.allowed_topics
            denied = policy.denied_topics
        else:
            allowed = policy.allowed_services
            denied = set()

        if resource in denied:
            self.violation_count += 1
            return False
        if '*' in allowed or resource in allowed:
            return True

        if self.default_policy == 'deny':
            self.violation_count += 1
            return False
        return True

    def load_default_policies(self):
        """Load the same default policies as the actual node."""
        self.add_policy(_AccessPolicy(
            role='operator',
            allowed_topics={'/miracle/+/state', '/miracle/system_kpis'},
            allowed_services={'/miracle/mes/submit_task'},
        ))
        self.add_policy(_AccessPolicy(
            role='engineer',
            allowed_topics={'*'},
            allowed_services={'*'},
        ))
        self.add_policy(_AccessPolicy(
            role='viewer',
            allowed_topics={'/miracle/+/state', '/miracle/system_kpis'},
        ))


@pytest.fixture
def enforcer():
    """Return an enforcer with default MIRACLE policies loaded."""
    e = _AccessEnforcer(default_policy='deny')
    e.load_default_policies()
    return e


class TestOperatorAccess:
    """Tests for the operator role."""

    def test_operator_can_read_state(self, enforcer):
        """Operators should be able to read machine state topics."""
        assert enforcer.check_access('operator', '/miracle/+/state', 'topic') is True

    def test_operator_can_read_kpis(self, enforcer):
        """Operators should be able to read system KPIs."""
        assert enforcer.check_access('operator', '/miracle/system_kpis', 'topic') is True

    def test_operator_can_submit_task(self, enforcer):
        """Operators should be able to call the submit_task service."""
        assert enforcer.check_access('operator', '/miracle/mes/submit_task', 'service') is True

    def test_operator_denied_security_topic(self, enforcer):
        """Operators should NOT access security alerts."""
        assert enforcer.check_access('operator', '/miracle/security/alerts', 'topic') is False

    def test_operator_denied_unlisted_service(self, enforcer):
        """Operators should NOT access services not in their policy."""
        assert enforcer.check_access('operator', '/miracle/ai/retrain', 'service') is False


class TestEngineerAccess:
    """Tests for the engineer role (wildcard access)."""

    def test_engineer_can_access_any_topic(self, enforcer):
        """Engineers with wildcard should access any topic."""
        assert enforcer.check_access('engineer', '/miracle/security/alerts', 'topic') is True
        assert enforcer.check_access('engineer', '/some/random/topic', 'topic') is True

    def test_engineer_can_access_any_service(self, enforcer):
        """Engineers with wildcard should access any service."""
        assert enforcer.check_access('engineer', '/miracle/ai/retrain', 'service') is True

    def test_engineer_wildcard_topic(self, enforcer):
        """Engineer policy should contain the wildcard '*'."""
        policy = enforcer.policies['engineer']
        assert '*' in policy.allowed_topics


class TestViewerAccess:
    """Tests for the viewer role (read-only subset)."""

    def test_viewer_can_read_state(self, enforcer):
        """Viewers should be able to read state topics."""
        assert enforcer.check_access('viewer', '/miracle/+/state', 'topic') is True

    def test_viewer_can_read_kpis(self, enforcer):
        """Viewers should be able to read KPIs."""
        assert enforcer.check_access('viewer', '/miracle/system_kpis', 'topic') is True

    def test_viewer_denied_services(self, enforcer):
        """Viewers should NOT have access to any services."""
        assert enforcer.check_access('viewer', '/miracle/mes/submit_task', 'service') is False

    def test_viewer_denied_unlisted_topics(self, enforcer):
        """Viewers should NOT access topics not in their allowed set."""
        assert enforcer.check_access('viewer', '/miracle/security/alerts', 'topic') is False


class TestUnknownRoleAndDefaultPolicy:
    """Tests for unknown roles and the default deny/allow policy."""

    def test_unknown_role_denied_by_default(self, enforcer):
        """An unknown role should be denied under the default 'deny' policy."""
        assert enforcer.check_access('hacker', '/miracle/+/state', 'topic') is False

    def test_unknown_role_allowed_when_default_allow(self):
        """An unknown role should be allowed when default policy is 'allow'."""
        e = _AccessEnforcer(default_policy='allow')
        assert e.check_access('unknown', '/anything', 'topic') is True

    def test_violation_count_increments(self, enforcer):
        """Each denied access should increment the violation counter."""
        initial = enforcer.violation_count
        enforcer.check_access('hacker', '/foo', 'topic')
        enforcer.check_access('hacker', '/bar', 'topic')
        assert enforcer.violation_count == initial + 2


class TestDeniedTopics:
    """Tests for explicit topic denial."""

    def test_explicitly_denied_topic(self):
        """A topic in the denied set should be blocked even if role exists."""
        e = _AccessEnforcer(default_policy='allow')
        e.add_policy(_AccessPolicy(
            role='restricted',
            allowed_topics={'/miracle/+/state'},
            denied_topics={'/miracle/security/secrets'},
        ))
        assert e.check_access('restricted', '/miracle/security/secrets', 'topic') is False

    def test_denied_takes_precedence_over_allowed(self):
        """Deny rules should take precedence over allow rules."""
        e = _AccessEnforcer(default_policy='allow')
        e.add_policy(_AccessPolicy(
            role='mixed',
            allowed_topics={'/miracle/data'},
            denied_topics={'/miracle/data'},
        ))
        assert e.check_access('mixed', '/miracle/data', 'topic') is False
