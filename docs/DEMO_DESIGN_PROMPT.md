# 🎯 ROBOT SCENARIO TESTING PLATFORM - DESIGN PROMPT

**Date:** 2025-10-19
**Purpose:** Comprehensive prompt for creating an interactive robot testing platform
**Target:** Web-based demonstration and testing interface

---

## 📋 PROJECT REQUIREMENTS

### **Primary Objective**
Create an interactive web-based platform where users can:
1. Select different robot types from a comprehensive list
2. Choose realistic test scenarios appropriate for each robot
3. Execute tests with live visualization
4. Monitor real-time performance metrics
5. View detailed test results with pass/fail criteria

### **Key User Story**
*"As a robotics engineer, I want to test different robots in realistic scenarios so that I can evaluate their performance, compare capabilities, and validate specifications before deployment."*

---

## 🎨 DESIGN SPECIFICATIONS

### **1. Visual Design Requirements**

#### **Color Scheme**
```css
Primary Colors:
- Primary Blue: #2563eb
- Secondary Purple: #7c3aed
- Success Green: #10b981
- Warning Orange: #f59e0b
- Danger Red: #ef4444
- Info Cyan: #06b6d4

Background:
- Dark Navy: #0f172a
- Medium Navy: #1e293b
- Light Slate: #334155

Text:
- Primary: #e2e8f0 (light gray)
- Secondary: #94a3b8 (medium gray)
- Tertiary: #64748b (darker gray)
```

#### **Typography**
- **Body Font:** Inter (Google Fonts)
- **Code Font:** JetBrains Mono
- **Base Size:** 16px
- **Heading Weights:** 600-700

#### **Layout Structure**
```
┌────────────┬───────────────────────────────────────────┐
│            │                                           │
│  Sidebar   │           Main Content Area              │
│  280px     │                                           │
│  Fixed     │  - Header Section                         │
│  Left      │  - Robot & Scenario Selection             │
│            │  - Test Execution Area                    │
│            │  - Visualization + Metrics                │
│            │  - Test Results                           │
│            │                                           │
└────────────┴───────────────────────────────────────────┘
```

---

## 🤖 ROBOT DATABASE STRUCTURE

### **Robot Data Model**
```javascript
{
    'robot_id': {
        name: 'Robot Name',
        manufacturer: 'Company Name',
        icon: '🤖',  // Emoji icon
        category: 'industrial|collaborative|mobile|humanoid|medical|drone|warehouse',
        scenarios: ['scenario_id_1', 'scenario_id_2', ...]  // Compatible scenarios
    }
}
```

### **Required Robot Types (Minimum 9)**
1. **Industrial Robots (2+)**
   - Example: FANUC M-10iA, ABB IRB 120
   - Use cases: Assembly, precision tasks, quality inspection

2. **Collaborative Robots (1+)**
   - Example: Universal Robots UR5e
   - Use cases: Human collaboration, safe interaction

3. **Mobile Robots (2+)**
   - Example: Boston Dynamics Spot, MiR100
   - Use cases: Navigation, inspection, transport

4. **Humanoid Robots (1+)**
   - Example: Honda ASIMO
   - Use cases: Social interaction, stair climbing

5. **Medical Robots (1+)**
   - Example: da Vinci Surgical System
   - Use cases: Surgical precision, tissue manipulation

6. **Drones (1+)**
   - Example: DJI Mavic 3
   - Use cases: Aerial mapping, package delivery

7. **Warehouse Robots (1+)**
   - Example: Amazon Kiva
   - Use cases: Shelf transport, order picking

---

## 🧪 TEST SCENARIO STRUCTURE

### **Scenario Data Model**
```javascript
{
    'scenario_id': {
        name: 'Scenario Name',
        description: 'Brief description of test (50-100 words)',
        icon: '🔧',  // Emoji icon
        difficulty: 'easy|medium|hard',
        duration: 8,  // seconds
        visualization: 'assembly|warehouse|surgical|flight',
        requirements: ['industrial', 'collaborative'],  // Compatible robot categories

        metrics: {
            metric_name: {
                target: 99.5,  // Target value
                unit: '%'      // Unit of measurement
            },
            // ... more metrics
        },

        testSteps: [
            'Step 1: Initialize and calibrate',
            'Step 2: Execute main operation',
            // ... 5-7 steps total
        ]
    }
}
```

### **Required Scenario Categories (21 Total)**

#### **Industrial Scenarios (4)**
1. **Assembly Line Production** (Medium, 8s)
   - Metrics: Accuracy 99.5%, Speed 120 parts/hr, Repeatability ±0.05mm
   - Steps: Initialize → Load → Align → Assemble → Verify → Place → Record

2. **High-Precision Pick & Place** (Hard, 6s)
   - Metrics: Accuracy 99.9%, Precision ±0.01mm, Speed 60 picks/hr
   - Steps: Vision locate → Grasp calculation → Approach → Pick → Transport → Place → Validate

3. **Automated Quality Inspection** (Medium, 7s)
   - Metrics: Defect detection 99.8%, Throughput 200 parts/hr
   - Steps: Capture images → AI detection → Measurement → Analysis → Classification → Logging

4. **High-Speed Operations** (Hard, 10s)
   - Metrics: Throughput 300 cycles/hr, Reliability 99.5%
   - Steps: Ramp up → Execute 1000 cycles → Monitor temperature → Track vibration → E-stop test

#### **Collaborative Scenarios (2)**
5. **Human-Robot Collaboration** (Medium, 9s)
   - Metrics: Safety 100%, Response time 50ms, Force limit 150N
   - Steps: Detect human → Reduce speed → Collaborate → Force test → E-stop → Resume

6. **Object Handoff** (Medium, 6s)
   - Metrics: Success 98%, Safety 100%, Response time 1.5s
   - Steps: Detect approach → Extend arm → Wait for grasp → Confirm → Release → Return

#### **Mobile Scenarios (4)**
7. **Warehouse Navigation** (Medium, 12s)
   - Metrics: Path accuracy 95%, Obstacle avoidance 100%, Speed 1.2 m/s
   - Steps: Start → Navigate 50m → Avoid obstacles → Retrieve → Navigate → Deliver → Return

8. **Complex Obstacle Course** (Hard, 15s)
   - Metrics: Completion time 180s, Collisions 0, Stability 95%
   - Steps: Stairs → Narrow passage → Moving obstacles → Climb → Slippery surface → Recover

9. **Facility Inspection** (Medium, 20s)
   - Metrics: Coverage 98%, Anomaly detection 95%, Duration 30 min
   - Steps: Generate route → Scan → Thermal imaging → Visual inspection → Gas detection → Upload

10. **Material Transport** (Easy, 10s)
    - Metrics: Efficiency 92%, Speed 1.5 m/s, Accuracy 95%
    - Steps: Receive request → Navigate → Load → Route → Transport → Unload → Confirm

#### **Medical Scenarios (3)**
11. **Surgical Precision Test** (Hard, 10s)
    - Metrics: Accuracy 99.99%, Precision ±0.1mm, Tremor <0.01mm
    - Steps: Calibrate → Identify → Incision → Suture → Tremor test → Collision avoid → Validate

12. **Automated Suturing** (Hard, 8s)
    - Metrics: Consistency 99%, Tension 0.5N, Spacing 3.0mm
    - Steps: Thread → Identify → Execute → Tension → Subsequent → Knot → Verify

13. **Tissue Manipulation** (Hard, 12s)
    - Metrics: Delicacy 99%, Force <0.05N, Damage 0%
    - Steps: Identify → Approach → Grasp → Retract → Hold → Release → Verify

#### **Drone Scenarios (2)**
14. **Aerial Mapping Survey** (Medium, 18s)
    - Metrics: Coverage 99%, Resolution 2 cm/px, Accuracy ±10cm
    - Steps: Pre-flight → Takeoff → Grid pattern → Images → Altitude → GPS → Landing

15. **Autonomous Package Delivery** (Hard, 12s)
    - Metrics: Accuracy ±1m, Reliability 99%, Speed 15 m/s
    - Steps: Load → Plan → Navigate → Avoid zones → Descend → Release → Return

#### **Warehouse Scenarios (3)**
16. **Shelf Transport Operation** (Medium, 10s)
    - Metrics: Lift 340kg, Stability 98%, Accuracy ±5mm
    - Steps: Locate → Align → Lift → Transport → Corners → Lower → Verify

17. **Order Picking Efficiency** (Medium, 15s)
    - Metrics: Accuracy 99.5%, Speed 100 items/hr, Error 0.5%
    - Steps: Receive → Route → Navigate → Identify → Pick → Repeat → Deliver

18. **Material Transport** (Easy, 10s)
    - Same as scenario 10 (reusable)

#### **Humanoid Scenarios (3)**
19. **Stair Climbing Challenge** (Hard, 8s)
    - Metrics: Stability 95%, Speed 0.3 m/s, Balance 98%
    - Steps: Approach → Detect → Calculate → First step → Climb 10 → Perturbation → Complete

20. **Human-Robot Object Handoff** (Medium, 6s)
    - Same as scenario 6 (reusable for humanoids)

21. **Social Interaction Test** (Medium, 10s)
    - Metrics: Speech recognition 95%, Response 2s, Gesture 90%
    - Steps: Greet → Listen → Process → Generate → Speak → Gestures → Eye contact

---

## 🎬 VISUALIZATION REQUIREMENTS

### **1. Assembly Line Visualization**
**Used for:** Industrial scenarios

**Elements:**
```html
- Animated conveyor belt (moving pattern)
- Workpiece moving across screen
- Robot arm with glowing joints
- Base platform
```

**Animations:**
- Conveyor: Continuous horizontal movement (3s loop)
- Workpiece: Left to right (5s linear)
- Joints: Pulse effect (2s infinite)

---

### **2. Warehouse Floor Visualization**
**Used for:** Mobile, warehouse scenarios

**Elements:**
```html
- Grid floor pattern (50px squares)
- 4 warehouse shelves (positioned at corners)
- Mobile robot (circular, gradient)
- Navigation path
```

**Animations:**
- Robot: Navigate between waypoints (8s ease-in-out)
- Path: Bezier curve following
- Shelves: Static with hover effect

---

### **3. Surgical Scene Visualization**
**Used for:** Medical scenarios

**Elements:**
```html
- Patient area (oval platform)
- Surgical target (pulsing dot)
- Precision instrument (thin line)
- Dark radial gradient background
```

**Animations:**
- Target: Pulse effect (1.5s)
- Instrument: Precision movement (3s)
- Lighting: Spotlight effect

---

### **4. Drone Flight Visualization**
**Used for:** Drone scenarios

**Elements:**
```html
- Flight zone (gradient sky background)
- Drone body (circular)
- 4 propellers (spinning)
- Delivery target point
```

**Animations:**
- Drone: Complex flight path (6s)
- Propellers: Fast rotation (0.3s)
- Target: Pulse effect (2s)

---

## ⚡ INTERACTIVE FUNCTIONALITY

### **1. Robot Selection**
```javascript
function selectRobot(robotId) {
    // Update selected state
    selectedRobot = robotId;

    // Highlight selected robot card
    updateRobotCardUI();

    // Filter and display compatible scenarios
    populateScenarios(robotId);

    // Log to console
    updateConsole('info', `Selected: ${robots[robotId].name}`);

    // Reset scenario selection
    selectedScenario = null;
}
```

**UI Updates:**
- Add 'selected' class to robot card
- Remove 'selected' from previous selection
- Enable scenario grid
- Clear previous scenario selection

---

### **2. Scenario Selection**
```javascript
function selectScenario(scenarioId) {
    // Update selected state
    selectedScenario = scenarioId;

    // Highlight selected scenario card
    updateScenarioCardUI();

    // Update visualization preview
    updateVisualization(scenarios[scenarioId].visualization);

    // Log to console
    updateConsole('info', `Scenario: ${scenarios[scenarioId].name}`);
    updateConsole('info', `Duration: ${scenarios[scenarioId].duration}s`);

    // Enable run button
    document.getElementById('runTestBtn').disabled = false;
}
```

**UI Updates:**
- Add 'selected' class to scenario card
- Show visualization preview
- Display scenario details
- Enable "Run Test" button

---

### **3. Test Execution**
```javascript
function runTest() {
    if (!selectedRobot || !selectedScenario) return;

    testRunning = true;
    const scenario = scenarios[selectedScenario];

    // Update UI state
    updateStatus('running', 'TEST IN PROGRESS');
    updateRunButton('running');

    // Log start
    updateConsole('success', '========== TEST STARTED ==========');

    // Execute test steps sequentially
    const stepDuration = (scenario.duration * 1000) / scenario.testSteps.length;
    let currentStep = 0;

    testInterval = setInterval(() => {
        if (currentStep < scenario.testSteps.length) {
            // Log current step
            updateConsole('info', `Step ${currentStep + 1}: ${scenario.testSteps[currentStep]}`);

            // Update metrics
            const progress = (currentStep + 1) / scenario.testSteps.length;
            updateMetrics(scenario.metrics, progress);

            currentStep++;
        } else {
            // Test complete
            completeTest(scenario);
        }
    }, stepDuration);
}
```

---

### **4. Real-Time Metrics Update**
```javascript
function updateMetrics(metrics, progress) {
    const metricsDisplay = document.getElementById('metricsDisplay');
    metricsDisplay.innerHTML = '';

    Object.entries(metrics).forEach(([key, data]) => {
        // Calculate current value with some randomness
        const value = data.target * (0.8 + Math.random() * 0.2) * progress;

        // Create metric display
        const metricHTML = `
            <div class="metric-item">
                <div class="metric-label">
                    <span>${formatMetricName(key)}</span>
                    <span class="metric-value">${value.toFixed(2)} ${data.unit}</span>
                </div>
                <div class="metric-bar">
                    <div class="metric-fill" style="width: ${(value/data.target)*100}%"></div>
                </div>
            </div>
        `;

        metricsDisplay.innerHTML += metricHTML;
    });
}
```

---

### **5. Test Completion**
```javascript
function completeTest(scenario) {
    clearInterval(testInterval);
    testRunning = false;

    // Determine success (90% success rate for demo)
    const success = Math.random() > 0.1;

    // Update status
    updateStatus(
        success ? 'passed' : 'failed',
        success ? 'TEST PASSED ✓' : 'TEST FAILED ✗'
    );

    // Log completion
    updateConsole(
        success ? 'success' : 'error',
        `========== TEST ${success ? 'PASSED' : 'FAILED'} ==========`
    );

    // Update results display
    displayResults(scenario, success);

    // Reset button
    updateRunButton('ready');
}
```

---

### **6. Console Logging**
```javascript
function updateConsole(type, message) {
    const console = document.getElementById('testConsole');
    const timestamp = new Date().toLocaleTimeString();

    const line = document.createElement('div');
    line.className = 'console-line';
    line.innerHTML = `
        <span class="timestamp">[${timestamp}]</span>
        <span class="console-${type}">${message}</span>
    `;

    console.appendChild(line);
    console.scrollTop = console.scrollHeight;  // Auto-scroll
}
```

**Console Types:**
- `info`: Blue - General information
- `success`: Green - Successful operations
- `warning`: Orange - Warnings
- `error`: Red - Errors

---

## 🎯 DIFFICULTY LEVEL SYSTEM

### **Easy 🟢**
**Characteristics:**
- Straightforward tasks
- Minimal precision required
- Standard operating conditions
- High success tolerance (>95%)

**Example Scenarios:**
- Material Transport
- Basic Navigation
- Simple Pick & Place

**Visual Indicator:**
```css
.difficulty-easy {
    background: rgba(16, 185, 129, 0.2);
    color: #10b981;
    border: 1px solid #10b981;
}
```

---

### **Medium 🟡**
**Characteristics:**
- Moderate complexity
- Good precision needed
- Some environmental challenges
- Standard success criteria (90-95%)

**Example Scenarios:**
- Warehouse Navigation
- Quality Inspection
- Human Collaboration
- Assembly Line Production

**Visual Indicator:**
```css
.difficulty-medium {
    background: rgba(245, 158, 11, 0.2);
    color: #f59e0b;
    border: 1px solid #f59e0b;
}
```

---

### **Hard 🔴**
**Characteristics:**
- High complexity
- Sub-millimeter precision
- Challenging conditions
- Strict success criteria (>98%)

**Example Scenarios:**
- Surgical Precision
- High-Speed Operations
- Obstacle Course
- Autonomous Delivery

**Visual Indicator:**
```css
.difficulty-hard {
    background: rgba(239, 68, 68, 0.2);
    color: #ef4444;
    border: 1px solid #ef4444;
}
```

---

## 📊 PERFORMANCE METRICS SYSTEM

### **Metric Categories**

#### **1. Accuracy Metrics**
```javascript
accuracy: {
    target: 99.5,
    unit: '%',
    description: 'Overall task accuracy',
    acceptable_range: [95, 100]
}
```

#### **2. Precision Metrics**
```javascript
precision: {
    target: 0.01,
    unit: 'mm',
    description: 'Position/placement precision',
    acceptable_range: [0, 0.05]
}
```

#### **3. Speed Metrics**
```javascript
speed: {
    target: 120,
    unit: 'parts/hr',
    description: 'Throughput or velocity',
    acceptable_range: [100, 150]
}
```

#### **4. Reliability Metrics**
```javascript
reliability: {
    target: 99.5,
    unit: '%',
    description: 'System uptime/success rate',
    acceptable_range: [98, 100]
}
```

#### **5. Safety Metrics**
```javascript
safety: {
    target: 100,
    unit: '%',
    description: 'Safety compliance score',
    acceptable_range: [100, 100]  // Must be 100%
}
```

---

## 🎨 CSS ANIMATION SPECIFICATIONS

### **1. Pulse Animation** (For glowing elements)
```css
@keyframes pulse {
    0%, 100% {
        box-shadow: 0 0 20px rgba(245, 158, 11, 0.6);
    }
    50% {
        box-shadow: 0 0 30px rgba(245, 158, 11, 1);
    }
}

.pulsing-element {
    animation: pulse 2s ease-in-out infinite;
}
```

---

### **2. Float Animation** (For hovering elements)
```css
@keyframes float {
    0%, 100% {
        transform: translateY(0px);
    }
    50% {
        transform: translateY(-10px);
    }
}

.floating-element {
    animation: float 3s ease-in-out infinite;
}
```

---

### **3. Rotate Animation** (For spinning elements)
```css
@keyframes rotate {
    from {
        transform: rotate(0deg);
    }
    to {
        transform: rotate(360deg);
    }
}

.rotating-element {
    animation: rotate 2s linear infinite;
}
```

---

### **4. Conveyor Animation** (For moving belts)
```css
@keyframes conveyor {
    from {
        background-position: 0 0;
    }
    to {
        background-position: 40px 0;
    }
}

.conveyor-belt {
    background: linear-gradient(90deg,
        #475569 25%, #334155 25%, #334155 50%,
        #475569 50%, #475569 75%, #334155 75%
    );
    background-size: 40px 60px;
    animation: conveyor 3s linear infinite;
}
```

---

### **5. Fade In Animation** (For content transitions)
```css
@keyframes fadeIn {
    from {
        opacity: 0;
        transform: translateY(20px);
    }
    to {
        opacity: 1;
        transform: translateY(0);
    }
}

.fading-element {
    animation: fadeIn 0.5s ease-out;
}
```

---

## 🔧 TECHNICAL IMPLEMENTATION

### **Technology Stack**
- **HTML5** - Structure
- **CSS3** - Styling and animations
- **JavaScript (ES6+)** - Interactivity
- **No frameworks required** - Pure vanilla JS

### **Browser Compatibility**
- Chrome 90+
- Firefox 88+
- Safari 14+
- Edge 90+

### **File Structure**
```
project/
├── robot_scenario_testing.html    # Main file (all-in-one)
└── assets/                         # Optional external assets
    ├── fonts/
    └── icons/
```

### **Performance Requirements**
- Initial load: < 2 seconds
- Animation frame rate: 60 FPS
- Memory usage: < 100 MB
- No lag during test execution

---

## 📱 RESPONSIVE DESIGN CONSIDERATIONS

### **Desktop (Primary Target)**
- Minimum width: 1280px
- Optimal: 1920px
- Sidebar: 280px fixed
- Main content: Fluid with max-width

### **Tablet (Future Enhancement)**
- Sidebar collapses to hamburger menu
- Grid layouts adjust to 2 columns
- Touch-friendly buttons (44px minimum)

### **Mobile (Future Enhancement)**
- Single column layout
- Bottom navigation
- Simplified visualizations
- Touch gestures for navigation

---

## ✅ ACCEPTANCE CRITERIA

### **Functional Requirements**
✅ User can select any robot from the list
✅ Only compatible scenarios appear for selected robot
✅ User can click "Run Test" to start execution
✅ Test executes step-by-step with visible progress
✅ Metrics update in real-time during test
✅ Console logs all test activities with timestamps
✅ Test completes with pass/fail result
✅ Results display with detailed metrics
✅ Visualizations animate appropriately for scenario type
✅ Status indicators update throughout test lifecycle

### **Visual Requirements**
✅ Modern, professional appearance
✅ Smooth animations (60 FPS)
✅ Consistent color scheme
✅ Clear visual hierarchy
✅ Readable typography
✅ Appropriate use of icons and emojis
✅ Loading states for all interactions
✅ Hover effects on interactive elements

### **Usability Requirements**
✅ Intuitive navigation and selection
✅ Clear feedback for all actions
✅ Obvious next steps at each stage
✅ Error prevention (disabled states)
✅ Helpful console messages
✅ Descriptive labels and tooltips
✅ Keyboard navigation support (arrows)

### **Performance Requirements**
✅ No lag during animations
✅ Instant UI updates on selection
✅ Smooth scrolling
✅ Responsive interactions
✅ No memory leaks during repeated tests

---

## 🎓 EDUCATIONAL VALUE

### **Learning Objectives**
1. **Robot Capabilities** - Understand what different robots can do
2. **Performance Metrics** - Learn industry-standard measurements
3. **Test Methodology** - See structured testing approaches
4. **Scenario Design** - Understand real-world test scenarios
5. **Results Analysis** - Interpret performance data

### **Target Audience**
- Robotics students
- Engineers evaluating robots
- Researchers comparing systems
- Industry professionals
- Technical decision-makers

---

## 🚀 FUTURE ENHANCEMENTS

### **Phase 2 Features**
1. **Custom Scenarios** - User-defined test creation
2. **Data Export** - CSV/PDF report generation
3. **Historical Tracking** - Save and compare test runs
4. **Multi-Robot Comparison** - Side-by-side testing
5. **Advanced Visualizations** - 3D graphics with Three.js

### **Phase 3 Features**
1. **Real Robot Integration** - Connect to actual robots via ROS2
2. **Cloud Storage** - Save results to database
3. **Collaboration** - Share scenarios with team
4. **AI Analysis** - Automated performance insights
5. **Mobile Apps** - Native iOS/Android versions

---

## 📄 DOCUMENTATION REQUIREMENTS

### **Code Documentation**
```javascript
/**
 * Executes a test scenario for the selected robot
 *
 * @param {string} robotId - The ID of the selected robot
 * @param {string} scenarioId - The ID of the test scenario
 * @returns {Promise<TestResult>} The test execution results
 *
 * @example
 * runTest('fanuc_m10ia', 'assembly')
 *   .then(result => console.log(result))
 *   .catch(error => console.error(error));
 */
```

### **User Documentation**
- Quick start guide
- Scenario descriptions
- Metric explanations
- Troubleshooting guide
- FAQ section

---

## 🎯 SUCCESS METRICS

### **User Engagement**
- Time spent on platform: > 5 minutes
- Number of tests run: > 3 per session
- Scenario exploration: > 50% scenarios tried
- Return rate: > 30% users return

### **Technical Performance**
- Page load time: < 2 seconds
- Animation frame rate: 60 FPS
- Zero JavaScript errors
- 100% mobile responsive

### **Educational Impact**
- User understanding increase: > 40%
- Confidence in robot selection: > 60%
- Satisfaction rating: > 4.5/5

---

## 📝 PROMPT SUMMARY

**TO CREATE THIS DEMO, IMPLEMENT:**

1. ✅ **Robot Database** - 9+ robots across 7 categories
2. ✅ **Scenario Library** - 21 realistic test scenarios
3. ✅ **Interactive Selection** - Click to select robot/scenario
4. ✅ **Live Execution** - Step-by-step test with animations
5. ✅ **Real-Time Metrics** - Performance data updating live
6. ✅ **4 Visualizations** - Assembly, Warehouse, Surgical, Flight
7. ✅ **Console Logging** - Timestamped event tracking
8. ✅ **Results Display** - Pass/fail with detailed metrics
9. ✅ **Professional UI** - Modern design with smooth animations
10. ✅ **Full Navigation** - Integration with existing pages

**RESULT:** A comprehensive, interactive robot testing platform that demonstrates realistic scenarios, tracks performance metrics, and provides educational value through engaging visualizations and detailed feedback.

---

**Status:** ✅ **PROMPT COMPLETE**
**Date:** 2025-10-19
**Version:** 1.0
**Use Case:** Create interactive robot scenario testing platforms
