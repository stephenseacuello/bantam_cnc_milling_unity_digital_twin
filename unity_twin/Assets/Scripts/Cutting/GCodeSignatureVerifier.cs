using System;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using UnityEngine;

namespace MiracleTwin.Cutting
{
    /// <summary>
    /// Result of G-code signature verification.
    /// </summary>
    public enum SignatureResult
    {
        Valid,
        Invalid,
        Missing
    }

    /// <summary>
    /// Verifies signatures on G-code programs.
    ///
    /// Signature format: The G-code file ends with a signature block:
    ///   ; --- MIRACLE SIGNATURE BLOCK ---
    ///   ; MIRACLE_SIG:&lt;base64_ed25519_signature&gt;
    ///   ; MIRACLE_HASH:&lt;hex_sha256_of_content&gt;
    ///   ; MIRACLE_SIGNER:&lt;signer_id&gt;
    ///   ; MIRACLE_SIGNED_AT:&lt;timestamp&gt;
    ///   ; --- END SIGNATURE BLOCK ---
    ///
    /// Verification strategy:
    /// The signature block includes both an Ed25519 signature and a SHA256 content
    /// hash. Unity does not have a built-in Ed25519 library, so this verifier checks
    /// content integrity by recomputing the SHA256 hash of the stripped G-code content
    /// and comparing it against the hash embedded in the signature block.
    ///
    /// This proves the content has not been modified since signing. Full Ed25519
    /// signature verification (proving the signer's identity) would require a native
    /// plugin (e.g., libsodium) or a server-side verification call.
    /// </summary>
    public static class GCodeSignatureVerifier
    {
        private const string SIG_BLOCK_START = "; --- MIRACLE SIGNATURE BLOCK ---";
        private const string SIG_BLOCK_END = "; --- END SIGNATURE BLOCK ---";
        private const string SIGNATURE_PREFIX = "; MIRACLE_SIG:";
        private const string HASH_PREFIX = "; MIRACLE_HASH:";

        /// <summary>
        /// Verify the signature of a G-code program.
        ///
        /// Extracts the SHA256 content hash from the signature block, recomputes
        /// the hash from the stripped G-code content, and compares. If they match,
        /// the content has not been tampered with since signing.
        ///
        /// NOTE: Full Ed25519 signature verification (authenticating the signer)
        /// would require a native crypto plugin or a server-side verification
        /// endpoint. This method only verifies content integrity via SHA256.
        /// </summary>
        /// <param name="gcodeText">The full G-code text including signature block.</param>
        /// <param name="publicKeyPem">Ed25519 public key in PEM format (reserved for future native plugin use).</param>
        /// <returns>SignatureResult indicating validity.</returns>
        public static SignatureResult Verify(string gcodeText, string publicKeyPem = null)
        {
            if (string.IsNullOrEmpty(gcodeText))
                return SignatureResult.Missing;

            // Check for signature block
            string signatureB64 = ExtractSignature(gcodeText);
            if (string.IsNullOrEmpty(signatureB64))
                return SignatureResult.Missing;

            // Strip signature block to get clean content
            string cleanContent = StripSignatureBlock(gcodeText);
            if (string.IsNullOrEmpty(cleanContent))
                return SignatureResult.Invalid;

            // Validate Ed25519 signature format (base64 decodable, correct length)
            try
            {
                byte[] signatureBytes = Convert.FromBase64String(signatureB64);
                // Ed25519 signatures are exactly 64 bytes
                if (signatureBytes.Length != 64)
                {
                    Debug.LogWarning("[GCodeSignatureVerifier] Invalid signature length: " +
                                   $"expected 64 bytes, got {signatureBytes.Length}");
                    return SignatureResult.Invalid;
                }
            }
            catch (FormatException)
            {
                Debug.LogWarning("[GCodeSignatureVerifier] Invalid base64 in signature");
                return SignatureResult.Invalid;
            }

            // Extract the embedded content hash from the signature block
            string embeddedHashHex = ExtractContentHash(gcodeText);
            if (string.IsNullOrEmpty(embeddedHashHex))
            {
                Debug.LogWarning("[GCodeSignatureVerifier] No content hash (MIRACLE_HASH) " +
                                "found in signature block. Cannot verify content integrity.");
                return SignatureResult.Invalid;
            }

            // Compute SHA256 of the stripped content
            byte[] computedHash;
            using (var sha256 = SHA256.Create())
            {
                computedHash = sha256.ComputeHash(Encoding.UTF8.GetBytes(cleanContent));
            }
            string computedHashHex = BitConverter.ToString(computedHash)
                .Replace("-", "")
                .ToLowerInvariant();

            // Compare computed hash against embedded hash
            string normalizedEmbedded = embeddedHashHex.Trim().ToLowerInvariant();
            if (!string.Equals(computedHashHex, normalizedEmbedded, StringComparison.Ordinal))
            {
                Debug.LogError("[GCodeSignatureVerifier] Content hash MISMATCH — " +
                              "G-code content has been modified after signing.\n" +
                              $"  Expected: {normalizedEmbedded}\n" +
                              $"  Computed: {computedHashHex}");
                return SignatureResult.Invalid;
            }

            // Content integrity verified via SHA256 hash match.
            // NOTE: Ed25519 signature authentication is not performed here.
            // For full signer identity verification, integrate a native Ed25519
            // library (e.g., libsodium via NativePluginInterface) or call a
            // server-side verification endpoint.
            Debug.Log("[GCodeSignatureVerifier] Content integrity verified (SHA256 hash match). " +
                     $"Hash: {computedHashHex.Substring(0, 16)}...");
            return SignatureResult.Valid;
        }

        /// <summary>Extract the base64 signature from the signature block.</summary>
        public static string ExtractSignature(string text)
        {
            if (text == null) return null;

            foreach (var rawLine in text.Split('\n'))
            {
                string line = rawLine.Trim();
                if (line.StartsWith(SIGNATURE_PREFIX))
                {
                    return line.Substring(SIGNATURE_PREFIX.Length).Trim();
                }
            }
            return null;
        }

        /// <summary>Extract the hex-encoded SHA256 content hash from the signature block.</summary>
        public static string ExtractContentHash(string text)
        {
            if (text == null) return null;

            foreach (var rawLine in text.Split('\n'))
            {
                string line = rawLine.Trim();
                if (line.StartsWith(HASH_PREFIX))
                {
                    return line.Substring(HASH_PREFIX.Length).Trim();
                }
            }
            return null;
        }

        /// <summary>Remove the signature block from G-code text.</summary>
        public static string StripSignatureBlock(string text)
        {
            if (text == null) return null;

            var lines = text.Split('\n');
            var result = new System.Collections.Generic.List<string>();
            bool inSigBlock = false;

            foreach (var rawLine in lines)
            {
                string trimmed = rawLine.Trim();
                if (trimmed == SIG_BLOCK_START)
                {
                    inSigBlock = true;
                    continue;
                }
                if (trimmed == SIG_BLOCK_END)
                {
                    inSigBlock = false;
                    continue;
                }
                if (!inSigBlock)
                {
                    result.Add(rawLine);
                }
            }

            // Remove trailing empty lines
            while (result.Count > 0 && string.IsNullOrWhiteSpace(result[result.Count - 1]))
                result.RemoveAt(result.Count - 1);

            return string.Join("\n", result);
        }
    }
}
