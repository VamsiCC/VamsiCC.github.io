/* ── Typing role animation ── */
const roles = [
  "ML ENGINEER",
  "ROBOTICS RESEARCHER",
  "COMPUTER VISION",
  "CONTROLS ENGINEER",
  "SYSTEMS BUILDER",
];
let roleIndex = 0;
let charIndex = 0;
let isDeleting = false;
const roleEl = document.getElementById("typing-role");
const typeSpeed = 70;
const deleteSpeed = 40;
const pauseTime = 2000;

function typeRole() {
  if (!roleEl) return;
  const current = roles[roleIndex];

  if (!isDeleting) {
    roleEl.textContent = current.substring(0, charIndex + 1);
    charIndex++;
    if (charIndex === current.length) {
      isDeleting = true;
      setTimeout(typeRole, pauseTime);
      return;
    }
    setTimeout(typeRole, typeSpeed);
  } else {
    roleEl.textContent = current.substring(0, charIndex - 1);
    charIndex--;
    if (charIndex === 0) {
      isDeleting = false;
      roleIndex = (roleIndex + 1) % roles.length;
      setTimeout(typeRole, 400);
      return;
    }
    setTimeout(typeRole, deleteSpeed);
  }
}
typeRole();

/* ── Active section tracking ── */
const sections = document.querySelectorAll(".section");
const navDots = document.querySelectorAll(".nav-dot");

const observer = new IntersectionObserver(
  (entries) => {
    entries.forEach((entry) => {
      if (entry.isIntersecting) {
        const id = entry.target.id;
        navDots.forEach((dot) => {
          dot.classList.toggle("active", dot.dataset.section === id);
        });
      }
    });
  },
  { threshold: 0.4 }
);
sections.forEach((s) => observer.observe(s));

/* ── Mobile menu ── */
const menuBtn = document.getElementById("menu-btn");
const mobileMenu = document.getElementById("mobile-menu");

menuBtn?.addEventListener("click", () => {
  mobileMenu.classList.toggle("open");
  document.body.style.overflow = mobileMenu.classList.contains("open") ? "hidden" : "";
});

mobileMenu?.querySelectorAll("a").forEach((link) => {
  link.addEventListener("click", () => {
    mobileMenu.classList.remove("open");
    document.body.style.overflow = "";
  });
});

/* ── Modal system ── */
const modalOverlay = document.getElementById("modal-overlay");
const modalBody = document.getElementById("modal-body");
const modalTitle = document.getElementById("modal-title");
const modalTag = document.getElementById("modal-tag");
const modalClose = document.getElementById("modal-close");

const projectData = {
  ekf: {
    tag: "Sensor Fusion · C++ · Eigen",
    title: "EKF Sensor Fusion",
    html: `
      <p>A 15-state Extended Kalman Filter that fuses high-rate IMU predictions with low-rate GPS position updates to produce smooth trajectory estimates. Built in C++ with Eigen, validated on KITTI driving data with an interactive MapLibre globe visualization.</p>

      <h4>State Vector</h4>
      <div class="math-block">
        <span class="var">x</span> = [ <span class="var">p</span><sub class="sub">x,y,z</sub> &nbsp;|&nbsp;
        <span class="var">v</span><sub class="sub">x,y,z</sub> &nbsp;|&nbsp;
        roll, pitch, yaw &nbsp;|&nbsp;
        <span class="var">b</span><sub class="sub">a</sub> &nbsp;|&nbsp;
        <span class="var">b</span><sub class="sub">g</sub> ]<sup>T</sup> &nbsp;∈ ℝ<sup>15</sup>
      </div>
      <p>Position and velocity in local ENU navigation frame. Attitude as ZYX Euler angles. Accelerometer and gyroscope biases estimated online in the body frame.</p>

      <h4>Prediction (IMU Propagation)</h4>
      <div class="math-block">
        <span class="var">p</span><sub class="sub">k+1</sub> = <span class="var">p</span><sub class="sub">k</sub> + <span class="var">v</span><sub class="sub">k</sub> · Δt<br>
        <span class="var">v</span><sub class="sub">k+1</sub> = <span class="var">v</span><sub class="sub">k</sub> + ( <span class="var">R</span>(φ,θ,ψ) · (<span class="var">a</span><sub class="sub">imu</sub> − <span class="var">b</span><sub class="sub">a</sub>) − <span class="var">g</span> ) · Δt<br>
        [φ, θ, ψ]<sub class="sub">k+1</sub> = [φ, θ, ψ]<sub class="sub">k</sub> + <span class="var">E</span>(φ,θ) · (<span class="var">ω</span><sub class="sub">imu</sub> − <span class="var">b</span><sub class="sub">g</sub>) · Δt
      </div>
      <p>Where <span class="var">R</span> maps body-frame accelerations to ENU, and <span class="var">E</span> maps body angular rates to Euler-angle rates. Covariance propagates via the analytical Jacobian <span class="var">F</span> = ∂f/∂<span class="var">x</span> with process noise <span class="var">Q</span>.</p>

      <h4>Update (GPS Correction)</h4>
      <div class="math-block">
        <span class="var">K</span> = <span class="var">P</span><sub class="sub">k</sub><sup>−</sup> <span class="var">H</span><sup>T</sup> ( <span class="var">H</span> <span class="var">P</span><sub class="sub">k</sub><sup>−</sup> <span class="var">H</span><sup>T</sup> + <span class="var">R</span><sub class="sub">gps</sub> )<sup>−1</sup><br>
        <span class="var">x</span><sub class="sub">k</sub> = <span class="var">x</span><sub class="sub">k</sub><sup>−</sup> + <span class="var">K</span> ( <span class="var">z</span><sub class="sub">gps</sub> − <span class="var">H</span> <span class="var">x</span><sub class="sub">k</sub><sup>−</sup> )<br>
        <span class="var">P</span><sub class="sub">k</sub> = ( <span class="var">I</span> − <span class="var">K</span><span class="var">H</span> ) <span class="var">P</span><sub class="sub">k</sub><sup>−</sup> ( <span class="var">I</span> − <span class="var">K</span><span class="var">H</span> )<sup>T</sup> + <span class="var">K</span> <span class="var">R</span><sub class="sub">gps</sub> <span class="var">K</span><sup>T</sup> &nbsp;<em>(Joseph form)</em>
      </div>
      <p>Only position is measured (3-DOF GPS), so <span class="var">H</span> = [<span class="var">I</span><sub>3×3</sub> | 0]. The Joseph form ensures numerical stability of the covariance update.</p>

      <h4>Demo Video</h4>
      <div class="video-embed" id="ekf-video-container">
        <div class="video-placeholder">
          <span>▶</span>
          <p>Add your demo video URL in index.html<br>(data-video-id attribute on the EKF card)</p>
        </div>
      </div>

      <p style="margin-top:1rem;">
        <a href="https://github.com/VamsiCC/EKF-Sensor-Fusion" target="_blank" rel="noopener" style="color:var(--primary);font-family:var(--font-mono);font-size:0.8rem;">View on GitHub →</a>
      </p>
    `,
  },

  drone: {
    tag: "Reinforcement Learning · Isaac Sim · Controls",
    title: "Autonomous Drone Racing",
    html: `
      <p>End-to-end autonomous drone racing through a multi-gate track, inspired by the Autonomous Drone Racing League. Trained a Proximal Policy Optimization (PPO) agent in NVIDIA Isaac Sim with 500 massively parallel physics environments, mapping raw kinematic state directly to rotor throttle commands via a rate controller.</p>

      <div class="stats-row">
        <div class="stat"><div class="stat-value">13.10s</div><div class="stat-label">Best Lap</div></div>
        <div class="stat"><div class="stat-value">~75%</div><div class="stat-label">Success Rate</div></div>
        <div class="stat"><div class="stat-value">500</div><div class="stat-label">Parallel Envs</div></div>
      </div>

      <h4>Sim-to-Real RL Architecture</h4>
      <ul>
        <li>Single neural network maps raw kinematic state to rotor throttle commands via PPO</li>
        <li>Predictive look-ahead observation for gate approach planning</li>
        <li>Low-level <strong>RateController</strong> converts policy outputs to thrust and body-rate commands</li>
        <li>Checkpoint resume and per-gate spawn for isolated training of hard track sections</li>
      </ul>

      <h4>MPCC-Inspired Contouring Control</h4>
      <p>The reward shaping draws directly from <strong>Model Predictive Contouring Control (MPCC)</strong> principles — minimizing contouring error along a reference path while maximizing progress:</p>
      <div class="math-block">
        <span class="var">r</span> = <span class="var">r</span><sub class="sub">progress</sub> + <span class="var">r</span><sub class="sub">waypoint</sub> + <span class="var">r</span><sub class="sub">speed</sub> + <span class="var">r</span><sub class="sub">gate_centering</sub><br>
        &nbsp;&nbsp;&nbsp;+ <span class="var">r</span><sub class="sub">crossing</sub> + <span class="var">r</span><sub class="sub">side_progress</sub> − <span class="var">r</span><sub class="sub">hover</sub> − <span class="var">r</span><sub class="sub">misalign</sub> − <span class="var">r</span><sub class="sub">collision</sub>
      </div>
      <ul>
        <li><strong>Gate centering</strong> — rewards centered, on-axis approach (contouring error minimization)</li>
        <li><strong>Waypoint shaping</strong> — curves trajectory onto gate centerline pre-crossing</li>
        <li><strong>Progress & speed</strong> — maximizes forward progress along the track contour</li>
        <li><strong>Anti-cheat penalties</strong> — reverse passage, hover, and misalignment penalties prevent reward hacking</li>
      </ul>

      <h4>Full Track Run</h4>
      <div class="video-embed" id="drone-video-container">
        <div class="video-placeholder">
          <span>▶</span>
          <p>Add your full lap video in index.html<br>(data-video-id attribute on the drone card)</p>
        </div>
      </div>

      <h4>Track Progression</h4>
      <p>Developed and validated across four track configurations: singular gate test, circular gate test, figure-8 layout, and the full 13-gate race track with stacked gates and altitude changes.</p>

      <h4>Extra Credit</h4>
      <ul>
        <li><strong>Payload racing</strong> — suspended sphere with bar collision detection (~45% success)</li>
        <li><strong>Moving gates</strong> — sinusoidal gate oscillation with velocity in observation space</li>
        <li><strong>Dynamic obstacles</strong> — kinematic sphere obstacles on the track</li>
      </ul>

      <p style="margin-top:1rem;">
        <a href="https://github.com/VamsiCC/A2RL-Drone-Course" target="_blank" rel="noopener" style="color:var(--primary);font-family:var(--font-mono);font-size:0.8rem;">View on GitHub →</a>
      </p>
    `,
  },

  trash: {
    tag: "Computer Vision · ESP32 · Hackathon",
    title: "TrashTalker",
    html: `
      <p><strong>Sodahacks 2025 Sustainability Track Winner</strong> — an intelligent waste classification system that uses AI and computer vision to automatically identify and categorize waste materials, helping reduce contamination in recycling streams.</p>

      <h4>How It Works</h4>
      <ul>
        <li>ESP32-CAM captures waste images in real time</li>
        <li>Images are JPEG-compressed and sent to OpenAI vision models via OpenRouter API</li>
        <li>AI classifies into categories: plastic, paper, metal, glass, organic, or other</li>
        <li>Results displayed on a web UI and archived to AWS S3</li>
      </ul>

      <h4>Tech Stack</h4>
      <ul>
        <li><strong>Hardware:</strong> ESP32-CAM, ESP32-S3</li>
        <li><strong>Firmware:</strong> C (ESP-IDF)</li>
        <li><strong>Backend:</strong> Python with AWS S3 bridge</li>
        <li><strong>AI:</strong> OpenAI Vision via OpenRouter API</li>
      </ul>

      <div class="stats-row">
        <div class="stat"><div class="stat-value">🥇</div><div class="stat-label">Sodahacks Winner</div></div>
        <div class="stat"><div class="stat-value">2–5s</div><div class="stat-label">Latency</div></div>
      </div>

      <p style="margin-top:1rem;">
        <a href="https://github.com/VamsiCC/Sodahacks_TrashTalker" target="_blank" rel="noopener" style="color:var(--primary);font-family:var(--font-mono);font-size:0.8rem;">View on GitHub →</a>
      </p>
    `,
  },
};

function openModal(projectId) {
  const data = projectData[projectId];
  if (!data) return;
  modalTag.textContent = data.tag;
  modalTitle.textContent = data.title;
  modalBody.innerHTML = data.html;
  modalOverlay.classList.add("open");
  document.body.style.overflow = "hidden";

  const card = document.querySelector(`[data-project="${projectId}"]`);
  const videoId = card?.dataset.videoId;
  const containerId = projectId === "ekf" ? "ekf-video-container" : projectId === "drone" ? "drone-video-container" : null;

  if (videoId && containerId) {
    const container = document.getElementById(containerId);
    if (container) {
      container.innerHTML = `<iframe src="https://www.youtube.com/embed/${videoId}" allowfullscreen loading="lazy" title="Project demo video"></iframe>`;
    }
  }
}

function closeModal() {
  modalOverlay.classList.remove("open");
  document.body.style.overflow = "";
}

document.querySelectorAll("[data-project]").forEach((card) => {
  card.addEventListener("click", () => openModal(card.dataset.project));
});

modalClose?.addEventListener("click", closeModal);
modalOverlay?.addEventListener("click", (e) => {
  if (e.target === modalOverlay) closeModal();
});
document.addEventListener("keydown", (e) => {
  if (e.key === "Escape") closeModal();
});

/* ── Scroll-triggered animations ── */
const animateEls = document.querySelectorAll(".section:not(#landing) .section-tag, .section:not(#landing) .section-title, .timeline-item, .skill-card, .project-card, .about-grid > *, .contact-grid > *");

const animObserver = new IntersectionObserver(
  (entries) => {
    entries.forEach((entry) => {
      if (entry.isIntersecting) {
        entry.target.style.animation = "fadeUp 0.6s ease forwards";
        animObserver.unobserve(entry.target);
      }
    });
  },
  { threshold: 0.15 }
);

animateEls.forEach((el) => {
  el.style.opacity = "0";
  animObserver.observe(el);
});
