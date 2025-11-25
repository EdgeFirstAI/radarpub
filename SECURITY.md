# Security Policy

Au-Zone Technologies takes security seriously across the entire EdgeFirst ecosystem, including RadarPub and the EdgeFirst Perception Middleware. This document outlines our security practices and how to report vulnerabilities responsibly.

## Supported Versions

We provide security updates for the following versions of RadarPub:

| Version | Support Status | End of Support |
|---------|----------------|----------------|
| 0.2.x   | ✅ Full support | Current stable |
| 0.1.x   | 🔒 Security fixes only | 2026-06-01 |
| < 0.1.0 | ❌ End of life | No support |

**Note:** As RadarPub is currently in pre-1.0 development, we recommend always using the latest release for production deployments.

## Reporting a Vulnerability

### How to Report

**For security vulnerabilities, please do NOT use public GitHub issues.** Instead, report privately using one of these methods:

#### Email (Preferred)

📧 **Email**: support@au-zone.com
**Subject**: `Security Vulnerability - RadarPub`

#### GitHub Security Advisory (Alternative)

Use GitHub's private vulnerability reporting feature:
https://github.com/EdgeFirstAI/radarpub/security/advisories/new

### What to Include

Please provide as much information as possible to help us assess and address the vulnerability:

1. **Description** - Clear explanation of the vulnerability
2. **Impact** - Potential security impact and affected components
3. **Steps to Reproduce** - Detailed reproduction steps or proof-of-concept
4. **Affected Versions** - Which versions are vulnerable
5. **Mitigation** - Suggested fixes or workarounds (if known)
6. **Discoverer** - Your name/handle for attribution (optional)

**Example Report:**

```text
Subject: Security Vulnerability - RadarPub CAN Frame Buffer Overflow

Description:
A buffer overflow vulnerability exists in the CAN frame parser when
processing malformed target data with oversized payloads.

Impact:
An attacker with access to the CAN bus could potentially crash the
radar node or execute arbitrary code by sending crafted CAN frames.

Steps to Reproduce:
1. Configure RadarPub to listen on CAN interface
2. Send CAN frame with ID 0x123 and 128-byte payload (exceeds spec)
3. Observer segmentation fault in parse_target_data()

Affected Versions:
0.1.0 through 0.2.3

Mitigation:
Add bounds checking in can.rs:parse_target_data() before memcpy operation.

Discoverer:
Jane Doe (jane@example.com)
```

### What to Expect

| Timeline | Action |
|----------|--------|
| **< 48 hours** | Initial acknowledgment of report |
| **< 7 days** | Initial assessment and severity classification |
| **Varies by severity** | Fix development and testing |
| **After fix** | Coordinated public disclosure |

### Response Timeline by Severity

| Severity | Target Fix Time | Criteria |
|----------|----------------|----------|
| **Critical** | 7 days | Remote code execution, authentication bypass, data breach |
| **High** | 30 days | Privilege escalation, significant data exposure, DoS |
| **Medium** | 90 days | Information disclosure, non-critical DoS |
| **Low** | Next release | Minor issues with limited impact |

## Responsible Disclosure

We ask that security researchers follow responsible disclosure practices:

### Do

- ✅ Allow reasonable time for us to develop and test fixes
- ✅ Avoid public disclosure until patches are available
- ✅ Notify us if the vulnerability becomes publicly known
- ✅ Work with us on coordinated disclosure timing
- ✅ Test on isolated systems (not production infrastructure)

### Don't

- ❌ Exploit vulnerabilities beyond proof-of-concept testing
- ❌ Access, modify, or delete data belonging to others
- ❌ Perform denial-of-service attacks
- ❌ Publicly disclose before coordinated disclosure date
- ❌ Violate privacy or disrupt services

## Recognition

We believe in recognizing security researchers who help improve our security:

### With your permission, we will

- 🏆 Credit you in security advisories
- 📝 Acknowledge you in release notes
- 🎖️ List you in our annual security acknowledgments
- 🔗 Link to your website/profile (if desired)

### Hall of Fame

We maintain a list of security researchers who have responsibly disclosed vulnerabilities:

- [To be established with first report]

## Security Best Practices for Deployment

### Recommendations for Production Use

**1. Network Isolation**

- Deploy RadarPub on isolated CAN networks
- Use firewall rules to restrict Zenoh traffic
- Implement network segmentation for sensor buses

**2. Access Control**

- Run RadarPub with minimal required privileges
- Use systemd security hardening directives
- Restrict CAN interface access to authorized processes

**3. Input Validation**

- Validate all CAN frames against protocol specifications
- Implement rate limiting for CAN message reception
- Monitor for anomalous sensor behavior

**4. Monitoring & Logging**

- Enable structured logging for security events
- Monitor for unexpected CAN traffic patterns
- Set up alerts for process crashes or restarts

**5. Updates & Patching**

- Subscribe to security advisories (see below)
- Apply security patches promptly
- Test updates in staging before production

**Example systemd hardening:**

```ini
[Service]
# Run as dedicated user
User=radarpub
Group=radarpub

# Security hardening
CapabilityBoundingSet=CAP_NET_RAW CAP_NET_ADMIN
NoNewPrivileges=true
PrivateTmp=true
ProtectSystem=strict
ProtectHome=true
ReadWritePaths=/var/lib/radarpub
RestrictAddressFamilies=AF_UNIX AF_INET AF_NETLINK AF_CAN
RestrictNamespaces=true
RestrictRealtime=true
SystemCallFilter=@system-service
SystemCallErrorNumber=EPERM
```

## Security Update Distribution

Security updates and advisories are distributed through:

### GitHub Security Advisories

- **Primary channel**: https://github.com/au-zone/radarpub/security/advisories
- **Automatic notifications**: Watch repository releases

### EdgeFirst Studio Notifications

For systems integrated with EdgeFirst Studio, security updates are distributed:

- Via Studio dashboard notifications
- Automatic update suggestions
- Impact assessment for deployed systems

### Mailing List

Subscribe to security announcements:

- **Email**: security-announce@au-zone.com (to be established)
- **Frequency**: Security advisories only (low volume)

### RSS Feed

- **GitHub Releases**: https://github.com/au-zone/radarpub/releases.atom

## Known Security Considerations

### CAN Bus Security

**Threat Model:**

- RadarPub operates on CAN bus networks which lack authentication and encryption by design
- Physical access to CAN bus allows frame injection and eavesdropping
- CAN bus is considered a trusted environment in automotive applications

**Mitigations:**

- Validate all incoming CAN frames against DRVEGRD protocol specification
- Implement anomaly detection for unexpected frame patterns
- Use physically isolated CAN networks for critical sensors
- Consider CAN bus gateway with filtering for defense-in-depth

### Zenoh Network Security

**Threat Model:**

- Zenoh publishes sensor data over IP networks
- Without TLS, data can be intercepted or modified

**Mitigations:**

- Use Zenoh TLS mode for encrypted communication (see configuration guide)
- Implement Zenoh access control for topic-level authorization
- Deploy on isolated sensor networks when possible
- Use VPN for wide-area deployments

### Supply Chain Security

**Dependencies:**

- RadarPub depends on open source Rust crates (see [NOTICE.md](NOTICE.md))
- Dependencies are audited using `cargo audit` in CI/CD
- SBOM (Software Bill of Materials) included with each release

**Verification:**

- Release artifacts are signed with GPG
- SHA256 checksums provided for all binaries
- SBOM available as `sbom.json` in release artifacts

## Security Audits

**Internal Audits:**

- Regular security reviews by Au-Zone engineering team
- Static analysis with Clippy and SonarQube
- Dependency vulnerability scanning with `cargo audit`

**External Audits:**

- [To be scheduled for v1.0 release]
- Contact support@au-zone.com for audit reports

## Additional Security Services

### For Production Deployments

Au-Zone Technologies offers enhanced security services for production customers:

**Security Audits & Compliance:**

- Custom security assessments
- Compliance certification assistance (ISO 26262, IEC 62443)
- Penetration testing coordination
- Threat modeling workshops

**Priority Security Support:**

- Expedited security patch delivery
- Private pre-disclosure for high-severity issues
- Dedicated security contact
- Custom hardening guidance

**Secure Development:**

- Security requirements analysis
- Secure integration consulting
- Custom security features

📧 **Contact**: support@au-zone.com for enterprise security services

## Scope

This security policy applies to:

- ✅ RadarPub core components (radarpub binary)
- ✅ Control utilities (drvegrdctl)
- ✅ Visualization tools (drvegrd-rerun)
- ✅ Published release artifacts

This policy does NOT cover:

- ❌ Third-party dependencies (report to upstream projects)
- ❌ EdgeFirst Studio (separate security policy)
- ❌ Operating system vulnerabilities (report to OS vendor)
- ❌ Hardware vulnerabilities (report to hardware vendor)

For vulnerabilities in the broader EdgeFirst ecosystem, please refer to the appropriate security policy for that component.

## Contact

- **Security reports**: support@au-zone.com (Subject: "Security Vulnerability")
- **General security questions**: support@au-zone.com
- **Commercial security services**: support@au-zone.com

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-01-15 | Initial security policy for open source release |

---

**Last Updated**: 2025-01-15
**Policy Owner**: Au-Zone Technologies Security Team
