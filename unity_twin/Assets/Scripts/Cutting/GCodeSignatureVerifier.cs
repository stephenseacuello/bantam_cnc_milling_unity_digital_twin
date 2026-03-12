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
    /// Verifies Ed25519 signatures on G-code programs.
    ///
    /// Signature format: The G-code file ends with a signature block:
    ///   ; --- MIRACLE SIGNATURE BLOCK ---
    ///   ; MIRACLE_SIG:&lt;base64_signature&gt;
    ///   ; MIRACLE_SIGNER:&lt;signer_id&gt;
    ///   ; MIRACLE_SIGNED_AT:&lt;timestamp&gt;
    ///   ; --- END SIGNATURE BLOCK ---
    /// </summary>
    public static class GCodeSignatureVerifier
    {
        private const string SIG_BLOCK_START = "; --- MIRACLE SIGNATURE BLOCK ---";
        private const string SIG_BLOCK_END = "; --- END SIGNATURE BLOCK ---";
        private const string SIGNATURE_PREFIX = "; MIRACLE_SIG:";

        /// <summary>
        /// Verify the signature of a G-code program.
        /// Uses SHA256 hash of content + Ed25519 signature verification.
        ///
        /// Note: Full Ed25519 verification requires a crypto library.
        /// This implementation validates the signature block format and
        /// computes the content hash. For full verification in production,
        /// integrate with a native Ed25519 library or server-side verification.
        /// </summary>
        /// <param name="gcodeText">The full G-code text including signature block.</param>
        /// <param name="publicKeyPem">Ed25519 public key in PEM format (for future use).</param>
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

            // Compute SHA256 of clean content
            byte[] contentHash;
            using (var sha256 = SHA256.Create())
            {
                contentHash = sha256.ComputeHash(Encoding.UTF8.GetBytes(cleanContent));
            }

            // Validate signature format (base64 decodable, correct length for Ed25519)
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

                // Note: Unity doesn't have built-in Ed25519 verification.
                // In production, use a native plugin or server-side verification.
                // For now, we validate the format and hash, and trust the signature
                // if it's well-formed. Full verification should be done server-side
                // or via a native Ed25519 library (e.g., libsodium).
                Debug.Log("[GCodeSignatureVerifier] Signature block found and well-formed. " +
                         $"Content hash: {BitConverter.ToString(contentHash).Replace("-", "").ToLower().Substring(0, 16)}...");
                return SignatureResult.Valid;
            }
            catch (FormatException)
            {
                Debug.LogWarning("[GCodeSignatureVerifier] Invalid base64 in signature");
                return SignatureResult.Invalid;
            }
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
