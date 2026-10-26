# GPU plane-sweep performance audit

Audit date: October 23, 2026. Repository HEAD: `673f898fc2812df4b0a78dc6bc1b99cddc840d89`.

This is a source-based audit with inspection of existing experiment artifacts. **No reconstruction benchmark or optimization was run.** The approximately one-hour plane-sweep result and CVP-MVSNet's approximately four-minute estimation and ten-minute fusion results are user-reported measurements. The user no longer has the one-hour run's configuration or console summary. Consequently, this report cannot assign percentages of that hour to individual stages or establish whether every reference actually used the GPU.

The implementation, shaders used by the application, and reconstruction parameters were left unchanged. The pre-existing modification to `GraphicsApp00/setup/setup.ini` was preserved. [The baseline manifest][manifest] records source, configuration, binary, and shader hashes; [the configuration snapshot][baselineini] preserves the current uncommitted settings. A separate audit-only GLSL compilation was compared with existing SPIR-V; it did not replace application assets. [Observations][observations] contains capability and input checks.

All source links below are absolute paths to the audited workspace. Line ranges in labels describe the inspected region; links open its first line.

## 1. Findings and limits of the evidence

**Verified:** the GPU conversion covers the per-reference depth search, reference texture rejection, cost acceptance, and confidence acceptance. It does **not** cover image decoding, source selection, depth-range estimation, point materialization, cross-view consistency, voxel fusion, or outlier removal. In successful `GPU` mode, the expensive CPU depth search is skipped. In `CPU_GPU_COMPARE`, it deliberately executes before the GPU, and CPU results remain authoritative. GPU failures can invoke the full CPU search. See [mode selection and fallback][M3955], [production GPU call][M4020], and [CPU postprocessing][M4188]. The comment describing CPU reconstruction at [Scene.cpp:1351][S1351] is stale; executable control flow is decisive.

The strongest avoidable costs are:

1. **Production diagnostic allocation and readback:** seven output buffers are allocated, transferred, mapped, and copied every reference. A texture diagnostic alone occupies 1,970,176,000 bytes at 1600 × 1184 and patch radius 2. Allocation and transfer happen even when texture rejection and CPU comparison are disabled. [Allocation sizes][G1987], [allocation loop][G2059], [copies and readback][G2073].
2. **Per-point camera sorting:** cross-view consistency allocates and sorts the same neighbor list for every candidate point, even though its ranking depends only on the reference camera. [MultiViewUtility.cpp:3131–3155][M3131].
3. **Repeated matching work:** each invocation serially visits every depth, every source, and every patch offset; reference sampling and five-iteration inverse distortion sit inside these loops. [Shader matching][C176], [unprojection][C151], [depth search][C246].
4. **No cross-reference image cache or persistent GPU resources:** source images are decoded again, uploaded at full input resolution, and destroyed per reference; compute pipeline and descriptors are also recreated. [Reference loop][M3870], [loader][M1733], [GPU setup][G2036], [pipeline creation][G2065], [cleanup][G2003].

These costs are verified in code. Their ordering by elapsed time remains a hypothesis until profiling. A large shader workload, CPU consistency, transfer overhead, and CPU fallback can each dominate different runs.

## 2. Current configuration is different from the reported benchmark

The parser assigns `values[key] = value` for each line; the **last duplicate wins**. [SetupUtility.cpp:205–239][U205]. The current file contains many successive experiment blocks. Its last active block and earlier settings resolve as follows; this is not a reconstruction of the historical one-hour settings.

| Setting | Current effective value | Evidence |
|---|---:|---|
| Display/execution | PlaneSweep / GPU | [INI:99,125][I99] |
| Reference step / point step | 1 / 1 | [INI:242–246][I242] |
| Sources / planes | 4 / 48 | [INI:243–244][I242] |
| Image scale / patch radius | 0.39 / 2 | [INI:242–249][I242] |
| Minimum valid sources / cost cutoff | 2 / 0.14 | [INI:249–251][I249] |
| Minimum opaque patch fraction / alpha cutoff | 0.75 / 0.01 | [INI:167–168][I167], [INI:224][I217] |
| Confidence rejection | on; difference 0.02 | [INI:254–255][I254] |
| Texture rejection | off | [INI:231][I228] |
| Cross-view consistency | on; 4 checked; 3 agreements; tolerance 0.05 | [INI:258–261][I258] |
| Voxel fusion / outlier removal | off / off | [INI:267–268][I264] |
| Sparse depth range / depth saving / experiments | SparsePoints / on / off | [INI:238–270][I238] |

Read-only input checks found 300 PNGs in the selected `Input` folder, all 4032 × 3040; COLMAP's header lists 299 registered images and 299 intrinsic records. The extra image file does not add a reference: iteration uses loaded COLMAP poses. The actual current processing grid is `lround(4032 × .39)` by `lround(3040 × .39)` = **1572 × 1186**, containing 1,864,392 samples at point step 1. Grid dimensions come from decoded image dimensions, not a fixed 1600 × 1184 setting. [M3992–4011][M3992].

**Correction after directory recheck:** the configured mask folder contains **300 PNG files**, `img_000.png` through `img_299.png`. All 299 registered image filenames have a same-name mask file. The initial directory query returned zero; that result was not corroborated and must not be used to infer missing masks. The reason for the discrepancy is unestablished. Decoding RGB images `img_000.png`, `img_149.png`, and `img_298.png` found their alpha above the cutoff, but that does **not** establish lack of foreground rejection: when a mask loads, its alpha or luminance replaces the sampled RGB alpha. Mask decoding success, mask coverage and runtime reference-invalid counts still need verification. [Mask loading and sampling][M1761], [Shader mask sampling][C38], [Observation record][observations].

`nvidia-smi` identified an RTX 4090, driver 617.42, and 24,564 MiB reported memory. `vulkaninfo` reported `maxStorageBufferRange = 4,294,967,295`, `timestampPeriod = 1 ns`, and `timestampComputeAndGraphics = true`. These are capability observations, not utilization or throughput measurements. [Observation record][observations].

### Shader and executable identity matter

Production loads `shaders/planeSweepDepthMapSmokeTest.comp.spv`; despite its name, this is the real production depth shader. [G2065][G2065]. `ReadShaderFile` first tries an executable-adjacent `shaders` directory, then configured shader output, then working-directory paths. [G391–432][G391]. Startup compilation writes each source to configured output and normally skips unchanged timestamps. [ShaderUtility.cpp:25–54][H25]. Current configuration enables this and selects `Output/Shaders`. [INI:21–31][I21].

The checked-in `Shaders/...comp.spv` differs from `Output/Shaders/...comp.spv`. Recompiling current GLSL with the installed `glslc` into the audit folder produced a **byte-identical hash to `Output/Shaders/...comp.spv`**. Both binaries disassemble to 8 × 8 × 1 workgroups; the checked-in copy has an older texture-function signature. The output shader matches the current source, but the shader loaded by a historical executable cannot be inferred without its resolved path and hash. [Hashes][manifest], [observations][observations].

The workspace has a Debug executable and older Release normal/trace executables. CMake's `out/build/x64-Debug/CMakeCache.txt:21–36` specifies `/Od /Ob0 /RTC1`; the phase2pb normal and trace cache files specify Release. Debug builds request validation layers when available; Release builds do not. [GraphicsUtility.h:246–250][V246]. This makes build identity a mandatory profiling field; it is not proof that the reported GPU run was Debug. Startup shader invocation supplies no `-O` optimization flag. [ShaderUtility.cpp:350–356][H350]. Evaluate shader compiler optimization only as a separately validated candidate.

## 3. Complete execution flow

### 3.1 Entry, startup, and reference preparation

1. `wWinMain` constructs `EngineCore` with `GraphicsApp00/setup/setup.ini`. Setup resolution checks the supplied path and walks up to six current-directory ancestors. `EngineCore` loads setup, configures paths, compiles startup shaders, and initializes graphics. [WinMain.cpp:30][W30], [SetupUtility.cpp:168–201][U168], [EngineCore.cpp:274–282][E274]. Startup compilation is outside the plane-sweep timer. This audit reports the settings in the inspected workspace file; a runtime capture must confirm the resolved file.
2. `Scene::Init` loads COLMAP intrinsics/poses using `MultiViewUtility::LoadColmapCameras`, then calls `Scene::DrawColmap`. [Scene.cpp:129–130][S129]; camera parsing and coordinate conversion are in [MultiViewUtility.cpp:589–737][M589]. All loaded registered poses remain eligible; `DisplayEveryNthCamera` subsamples camera visualization, whereas plane-sweep reference selection uses its own reference step.
3. `DrawColmap` selects the PlaneSweep display branch, starts the wall timer, constructs `PlaneSweepGpu`, and initializes it with the **graphics queue and existing command pool**. Initialization only stores handles; it does not build a persistent pipeline or invoke smoke tests. [Scene.cpp:1156–1169][S1156], [PlaneSweepGpu.cpp:443–456][G443].
4. Normal operation calls `BuildColmapPlaneSweepPointCloud(..., nullptr, planeSweepGpuPtr)`. The metrics pointer is null. The optional experiment branch instead runs multiple changed parameter configurations and passes metrics; it exits before reaching the normal branch. **Do not use experiment mode to profile an unchanged baseline.** [Scene.cpp:1173–1187][S1173], [1347–1353][S1347].
5. `BuildColmapPlaneSweepPointCloud` checks readiness; makes filename lookups once for images/masks; loads sparse points/tracks once; builds camera-ID and sparse-point lookups once. [M3724–3753][M3724]. Directory indexing is not an image-content cache. Sparse loading is text parsing, applies `DisplayEveryNthPoint`, and deduplicates image IDs within each track. [M1391–1471][M1391].
6. The reference loop visits camera indices `0, referenceStep, ...`. With 299 poses and step 1, at most 299 references execute. Failures to load or obtain sufficient sources reduce this number. [M3870–3913][M3870].
7. For each reference, `LoadPlaneSweepImage` finds intrinsics, decodes the image, optionally decodes a mask, scans mask alpha to choose alpha-versus-luminance opacity, and checks whether any opaque pixel exists. All of this is CPU work. `LoadTexture` creates a WIC factory/decoder/converter and copies full RGBA pixels on each call. [M1733–1813][M1733], [TextureUtility.cpp:232–279][T232].
8. `SelectSourceCameraIndices` counts shared sparse observations, computes geometric scores for all other cameras, sorts by shared-point count then distance/alignment, and returns up to `4 × requestedSourceCount` candidates. Images are loaded in order until the requested number succeed. Actual GPU work uses the number successfully loaded, which can be less than requested if still above the required minimum. [M1818–1890][M1818], [M3890–3913][M3890].
9. `EstimateDepthRange` projects sparse points into the reference camera and takes the rounded 5th/95th percentiles with 10% span expansion; fewer than 12 valid samples uses manual near/far fallback. `PercentileValue` sorts the same vector twice. [M1894–1945][M1894], [M3357–3366][M3357].
10. The single-pixel debug route can run diagnostics and return without full reconstruction if enabled for the selected image. The compile-time trace macro changes this gating. [M3940–3947][M3940]. It must be recorded separately from production timing.

### 3.2 Execution modes and the CPU baseline

| Mode | Depth-search behavior | Authoritative downstream output |
|---|---|---|
| `CPU_ONLY` | `BuildReferencePoints` on CPU; no production compute dispatch | CPU depth and CPU candidates |
| `GPU` | `RunProductionDepthMap`; CPU search skipped on success | GPU depth, then CPU candidates |
| `CPU_GPU_COMPARE` | CPU search first, GPU search second, CPU/GPU map comparison afterward | CPU depth and CPU candidates |
| `GPU` failure | Log reason and run full CPU depth search; a detected device loss disables further GPU attempts in this call | CPU fallback output |

Evidence: [M3955–3968][M3955], [M3970–4098][M3970]. More than eight loaded sources is unsupported by this GPU implementation and triggers fallback; four sources are supported. CPU-side allocations before the GPU function's internal `try` can also throw; the caller catches exceptions. A long CPU fallback can make a nominally GPU run take much longer. Verify attempted/succeeded/fallback counts before interpreting utilization.

CPU `BuildReferencePoints` allocates a full zero-filled scaled depth map, iterates sampled grid points, rejects invalid/transparent centers and optionally low texture, loops through all inverse-depth planes, calculates RGB mean absolute difference costs, tracks best/second best, and applies thresholds. It then generates points and saves depth. [M2950–3084][M2950]. `ComputePlaneCost` contains the source/patch loops and original CPU projection/sampling implementation. [M2063–2232][M2063]. This remains the numerical baseline.

### 3.3 Production GPU preparation, submission, and readback

`PlaneSweepGpu::RunProductionDepthMap` performs the following synchronously, once per reference:

| Step | Location | Executing processor |
|---|---|---|
| Validate dimensions, source count and RGBA layout; calculate grid/workload; format input log | [G1929–1960][G1929] | CPU |
| Build 18 image slots: reference RGB/mask, eight sources, eight masks; unused slots use 1 × 1 dummies | [G1961–1979][G1961] | CPU |
| Pack camera poses, intrinsics, runtime parameters and mask metadata | [G1980–1991][G1980] | CPU |
| Allocate and zero large host diagnostic vectors | [G1987–1993][G1987] | CPU |
| Allocate image staging, map/copy/unmap full input images; create device images/views | [G2036–2054][G2036] | CPU/API plus host memory copies |
| Create nearest sampler, four input staging/device pairs, seven output/readback pairs | [G2057–2059][G2057] | CPU/API |
| Create descriptor layout/pool/set; read SPIR-V; create shader/pipeline/layout with no supplied pipeline cache | [G2060–2065][G2060] | CPU/API/driver |
| Record image transitions, 18 image uploads, four input-buffer copies and barriers | [G2067–2070][G2067] | CPU recording; GPU transfer execution |
| Zero full depth output and establish transfer-to-compute visibility | [G2071][G2071] | GPU transfer |
| **One compute dispatch for the entire reference** | [G2072][G2072] | GPU compute |
| Compute-to-transfer barriers, seven full output copies to host-visible buffers, transfer-to-host barriers | [G2073–2074][G2073] | GPU transfer/synchronization |
| Submit once; wait on a fence with infinite timeout | [G2079–2085][G2079] | CPU submit and blocking wait |
| Allocate host destinations, map/copy/unmap all seven outputs, copy depth/status into result vectors, count accepted/nonfinite depths | [G2088–2104][G2088] | CPU |
| Destroy all per-reference resources | [G2003–2013][G2003], [G2105][G2105] | CPU/API |

Buffers holding matching inputs and outputs are device-local. Host-visible coherent buffers serve staging/readback. There is no evidence that the production shader directly reads all pixels from host memory. Coherence removes explicit flush/invalidate requirements for these selected memory types, but not copying or synchronization costs. Images are optimal tiled `R8G8B8A8_UNORM`, with one mip level. [G2038–2059][G2036].

There is **no `vkQueueWaitIdle` on the normal production depth-search path**. Its ordinary stall is `vkWaitForFences` once per reference. `vkDeviceWaitIdle` at [G2005][G2003] belongs to cleanup while submission is still outstanding, such as after an error; successful fence completion clears that flag. `vkQueueWaitIdle` at [G1682][G1682] belongs to a disabled texture smoke-test route. Do not charge every textual wait occurrence to production.

### 3.4 What the production shader actually does

The dispatch uses `(ceil(gridWidth/8), ceil(gridHeight/8), 1)` workgroups. [G2072][G2072]. Its GLSL workgroup is 8 × 8 × 1. [Shader:3][C3]. A single invocation owns a sampled reference pixel and executes all its planes, sources, and patch samples serially. It is not one invocation per depth hypothesis and does not materialize a full cost volume.

1. Map invocation coordinates through `pointStep` into the scaled grid and then by **integer division** into original-resolution image coordinates. [Shader:213–222][C213].
2. Sample reference RGB plus optional mask. Reject transparent/invalid centers **before** texture calculation and plane search. [Shader:226–246][C226].
3. If enabled, compute masked grayscale standard deviation over the patch; also write per-sample diagnostic records. [Shader:76–131][C76].
4. For every inverse-depth hypothesis, run `ComputePlaneCostCpuEquivalent`. [Shader:250–255][C246].
5. For every source and patch offset: resample reference RGB/mask, unproject the patch pixel with five distortion-correction iterations, transform/project into source with distortion, bounds-check, bilinearly sample source RGB/mask, and accumulate RGB MAD. A source contributes only if enough opaque paired samples survive. All valid source means are averaged; `MinValidSourceViews` is a minimum, **not** an instruction to stop after two good sources. [Shader:176–208][C176].
6. Track best and second-best valid costs across all planes, apply cost/confidence thresholds, write chosen camera-Z depth or zero. [Shader:272–282][C272].
7. Write seven mapping words, status, texture diagnostics, counts, and the last plane's source-0 sample count. [Shader:283–291][C283]. These counts are not a complete execution-work counter.

The shader uses manual bilinear sampling: four `texelFetch` operations followed by float interpolation. Masked sampling repeats that for the mask. The nearest sampler does **not** mean nearest-neighbor reconstruction: interpolation is explicit. [Shader:38–74][C38], [G2057][G2057]. The source-array switches use constant sampler indices; the source-loop index is normally uniform among active lanes, so the switch itself is not evidence of per-pixel divergence. [Shader:48–62][C48], [134–148][C134].

For current radius 2 and opaque ratio 0.75, each contributing source needs **at least 19 paired valid samples out of 25** at that depth; at least two sources must contribute. This threshold does not reduce the loop to 19 samples or two sources: the code completes all patch/source loops to form the averages. It also does not precheck whether the reference patch alone contains 19 valid opaque pixels before beginning the depth loop. [Shader:178–208][C176].

### 3.5 Depth filtering, points, consistency, fusion, and display

After successful GPU search, `BuildPlaneSweepPointCandidatesFromDepthMap` scans sampled entries on the CPU, ignores invalid depths, resamples color/alpha, unprojects each accepted depth again, and appends a world-space candidate with originating map index. [M2907–2946][M2907], [M4038–4047][M4038]. This is not GPU point generation.

`SaveDepthOutput` writes a width/height header plus float depth raster, scans depth min/max, creates a full grayscale RGBA preview and PNG-encodes it through WIC. It executes **before cross-view consistency and fusion**. Saved depth maps therefore do not reflect later point rejections. [M2236–2303][M2236]. CPU and GPU paths both save the authoritative map, once per reference.

Depth maps are retained only when cross-view consistency is enabled. [M4101–4104][M4101]. After all reference searches, CPU `ApplyCrossViewConsistency` evaluates each candidate against nearby retained maps. Its geometric neighbor ranking differs from source selection: it uses distance/alignment only, without shared sparse-track ranking. [M3103–3211][M3103]. It compares projected camera-Z to the best matching valid depth in a square neighborhood, not a full round-trip reprojection/normal test.

CPU `BuildFilteredPoints` either averages points/colors in voxel buckets and applies an observation minimum, or copies all candidates to points. It can then apply radius-based neighbor removal using spatial hash cells. [M3215–3346][M3215]. Current effective settings disable both fusion and outlier removal, so their expensive enabled branches cannot explain the current run. Cross-view consistency is enabled.

`Scene::AddColmapPointCloudObject` constructs the display object. Its initializer duplicates positions for picking and builds vertices and sequential indices; `SetMesh` also computes selection bounds. This CPU mesh work **is inside** the scene's reconstruction timer. [S1353–1365][S1351], [S1428–1440][S1428], [SceneObjectPointCloud.h:40–67,107–136][P40], [SceneObject.cpp:68–101][O68].

Actual GPU vertex/index upload happens later in `Scene::InitSceneObjects`, after `DrawColmap` returns, via `SceneObject::Init` → `CreateVertexBuffer`/`CreateIndexBuffer` → staging and `CopyBuffer` → `EndSingleTimeCommands`. These copies use `vkQueueWaitIdle`, but are **outside the printed plane-sweep timer** on this startup path. [Scene.cpp:171,2258–2263][S171], [O237–246][O237], [O1045–1123][O1045], [O1463–1472][O1463], [O1643–1658][O1643]. Interactive picking later scans points; it is not reconstruction matching.

## 4. Computational cost derived from the code

Define:

- `N`: loaded registered views; `R ≤ ceil(N/referenceStep)` successfully processed references.
- `W,H`: decoded original image size; `w = max(1,lround(W×scale))`, likewise `h`.
- `s = max(1,pointStep)`; `P = ceil(w/s) × ceil(h/s)` sampled reference centers; `E = w×h` allocated depth entries.
- `D = max(2,planeCount)`; `S`: successfully loaded sources; `A = (2×patchRadius+1)²`.
- `F ≤ P`: centers passing reference-alpha and optional texture rejection; `Q`: all accepted point candidates; `K`: retained maps; `C`: cross-view count.

The dominant matching loop has **exactly `F×D` plane-cost calls and `F×D×S×A` patch-loop iterations**, with projection/source fetches skipped conditionally. There is no early depth termination. Reference sampling is attempted inside every patch iteration; unprojection occurs only after that sample survives. Actual source samples can be much smaller than the upper bound. Counts below are **loop ceilings for all sampled centers entering search**, not observed GPU instructions or memory transactions. [M2984–3061][M2984], [Shader:183–208,250–274][C176].

For the reported comparison grid 1600 × 1184, dense point/reference steps, D=96 and S=4:

| Quantity | Per reference | Across 299 references |
|---|---:|---:|
| Reference centers `P` | 1,894,400 | 566,425,600 |
| Pixel/depth candidates `P×D` | 181,862,400 | 54,376,857,600 |
| Pixel/depth/source comparisons `P×D×S` | 727,449,600 | 217,507,430,400 |
| Patch iterations, radius 1, A=9 | 6,547,046,400 | 1,957,566,873,600 |
| Patch iterations, radius 2, A=25 | 18,186,240,000 | 5,437,685,760,000 |
| Patch iterations, radius 3, A=49 | 35,645,030,400 | 10,657,864,089,600 |

Historical patch radius and exact sampling are unavailable. Radius 2 is the **current** setting, not an assertion about the one-hour run. A fully evaluated radius-2 patch iteration uses eight explicit RGB texel fetches across reference/source; with both masks it can use sixteen. This gives ceilings of 145,489,920,000 or 290,979,840,000 explicit texel-fetch operations per reference before texture-stage work. Texture cache reuse, compiler optimization, bounds rejection, and alpha rejection mean these are not DRAM transaction counts. Five inverse-distortion iterations per surviving reference patch sample add up to 90,931,200,000 iteration bodies per reference in the all-valid radius-2 case.

Current effective settings instead give `P=1,864,392`, `D=48`, `S≤4`, `A=25`: at most **8,949,081,600 patch iterations per reference**. Point step 2 would reduce the number of matching centers by approximately four, not merely decimate final output; reference step 10 would attempt 30 rather than 299 views. These are complexity relationships, not proposed parameter changes. [M3748–3751][M3748], [M2984][M2984], [Shader:215][C213].

**Image scale controls the matching-center grid; it does not resize the RGB or masks.** Patch offsets are ±radius in original image pixels. Full 4032 × 3040 images are decoded/uploaded and sampled even for a smaller grid. Therefore a 5 × 5 patch here has different image support from a 5 × 5 patch after resizing to 1600 pixels wide. Treating grid scale as image resampling would change matching behavior. [M1761–1784][M1761], [M2988–2989][M2984], [Shader:190–193][C176], [G1966–1975][G1961].

### Other stage costs

| Stage | Code-derived work and memory | Source |
|---|---|---|
| Sparse parsing/indexing | Once per reconstruction; approximately linear in file tokens/observations, with per-track duplicate-search overhead | [M1391][M1391] |
| Source ranking | Per reference, shared-track visits plus `O(N log N)` sorting; not per pixel | [M1818][M1818] |
| Intrinsic lookup | Linear scan of intrinsic records for each image load | [M1320][M1320] |
| Depth range | `O(k)` projections plus two `O(k log k)` sorts per reference | [M1894][M1894], [M3357][M3357] |
| Decode/upload | Full image/mask pixels per reference/source role; no content reuse | [M1733][M1733], [G2036][G2036] |
| Reference texture test | Up to `P×A` masked samples, once per center, before depths | [C76][C76] |
| Point materialization | `O(P)` raster visits, `O(Q)` color samples/unprojections/appends | [M2907][M2907] |
| Depth saving | `O(E)` float write, min/max scan, preview construction, PNG encoding | [M2236][M2236] |
| Cross-view consistency | `O(Q[K log K + C(2 max(1,s)+1)²])`; per-point allocation plus sort | [M3103][M3103] |
| Voxel fusion | Expected `O(Q)` hash insertion plus occupied-voxel traversal; collision behavior and memory matter | [M3215][M3215] |
| Outliers | `O(M)` cell construction plus per-point occupants of 27 cells; worst case `O(M²)` | [M3281][M3281] |
| CPU display mesh | `O(finalPoints)` picking positions, vertices, indices, bounds | [P40][P40], [O68][O68] |

At `K=299`, each point rebuilds 298 camera scores and sorts them to keep at most four. For an **illustrative, unmeasured** `Q=10 million`, that is 2.98 billion candidate constructions and roughly 24.5 billion comparator-scale operations (`Q×298×log2(298)`), plus ten million vector allocations. Actual `std::sort` comparisons and accepted-point counts must be measured. Camera-neighbor precomputation reduces the repeated ranking component to `O(K² log K + Q×C)` without changing selected neighbors if original ordering/ties are preserved.

With cross-view enabled, 299 maps at 1600 × 1184 require **2,265,702,400 bytes (2.11 GiB)** for float depth payloads alone. Point candidates, consistency survivor vectors, fused points, hash nodes, display vertices, and picking positions add substantially. `PlaneSweepPointCandidate` holds an index, Float3, and Float4; measure `sizeof`/capacity in the actual build rather than assuming an ABI size. [MultiViewUtility.h:497–537][MH497]. RAM pressure and paging are plausible at dense completeness; they are unmeasured.

## 5. Production diagnostics: exact byte accounting

Host code sets `T = max(100,10+10A)` 32-bit words per sampled center for texture trace. Other diagnostic outputs total eleven words per center: mapping seven, status one, texture sample count one, minimum opaque count one, plane sample count one. Depth adds `4E` bytes. Thus total output/readback payload is:

`B = 4E + 4P(T+11)` bytes per reference.

This is independently allocated in device output and host-visible readback, then copied into CPU vectors. [G1987–1997][G1987], [G2059][G2059], [G2074][G2073], [G2088–2098][G2088].

| Radius | Trace stride T | Texture trace bytes, P=E=1,894,400 | All seven outputs, bytes/reference |
|---|---:|---:|---:|
| 1 | 100 | 757,760,000 | 848,691,200 |
| 2 | 260 | 1,970,176,000 | 2,061,107,200 |
| 3 | 500 | 3,788,800,000 | 3,879,731,200 |

For radius 2, **616,271,052,800 bytes** would be copied to readback across 299 equal-size references if all GPU calls succeed. This is payload arithmetic, not a PCIe measurement; staging-memory placement depends on driver/device memory types. Current 1572 × 1186 settings produce 1,938,967,680 trace bytes and 2,028,458,496 total output bytes per reference, independent of plane count.

The large zero-initialized CPU `textureTrace` exists before `gpuSetupStart`; a second large `gpuTextureTrace` destination is allocated during readback. They overlap in lifetime. Texture trace alone therefore accounts for two device/readback allocations and two host vectors—about **7.88 GB of logical allocations** at the radius-2 example, before other buffers/images/points. Residency and physical accounting depend on the driver and operating system. [G1987–2015][G1987], [G2088][G2088].

The shader writes detailed texture records only when its texture test executes. If disabled, it writes a small summary slot; most of that allocated output is unwritten. Nevertheless, host code copies the **entire buffer**. `compareCpuResult=false` is tested only **after** allocation, transfer, map, and memcpy. [Shader:230–244,288][C226], [G2099–2106][G2099]. The ordinary C++ call always passes false, including external CPU/GPU-compare mode. [M4020–4024][M4020].

The optional `GRAPHICSAPP_ENABLE_PHASE_2PB_GPU_TRACE` controls an additional trace binding and block. It does **not** disable these always-declared production diagnostics. A shader built with that macro additionally declares binding 15 and reads metadata beyond the normal 18 words, while current production setup binds only 0–14. Treat trace-shader/build mixing as a separate compatibility issue, not a performance remedy. [Shader:24–26,257–270][C24], [G1965][G1961], [G2060][G2060].

Check buffer descriptor ranges against `maxStorageBufferRange`: current setup checks descriptor counts but not this range. Radius 3 at this grid fits the inspected device's approximately 4 GiB range; larger patches can exceed it. This does not establish an error at the reported radius, which is unknown. [G2018–2020][G2018], [G2064][G2060]; [Vulkan limits definition](https://docs.vulkan.org/refpages/latest/refpages/source/VkPhysicalDeviceLimits.html).

## 6. GPU utilization and repeated computations

At 1600 × 1184, step 1, dispatch is 200 × 148 × 1 = **29,600 workgroups, 1,894,400 invocations**. That is substantial available parallel work; source inspection does not support a claim that the GPU is starved merely because there is one dispatch. At current dimensions, dispatch is 197 × 149 = 29,353 groups with bounds guards on padded invocations. Actual occupancy, eligible warps, cache behavior, register pressure, clocks, and warp efficiency need GPU profiling.

**Verified implementation properties and implications to test:**

- Each thread has a long serial `D×S×A` workload and best/second-best dependence across planes. No shared-memory tile, subgroup sharing, plane-parallel reduction, or persistent reference-ray cache appears in the production shader. This can raise instruction cost; actual register allocation or spills cannot be inferred from GLSL alone. [C176][C176], [C246][C246].
- Adjacent invocations cover nearby centers, so reference/source image cache reuse is possible. Projected source coordinates may be irregular; rejection branches vary at silhouettes, occlusions, image boundaries and invalid depths. Such branches can leave some lanes inactive while others complete the full search. Measure active-lane efficiency rather than assuming a fixed penalty. [C191–205][C176], [C226–246][C226].
- The five-iteration inverse-distortion solution depends on reference pixel and calibration, **not depth or source**. The code recomputes it for each depth/source/patch sample. Reference color, reference opacity, mask-coordinate scaling, minimum-opaque threshold, and depth-plane values also have reuse opportunities. [C151–180][C151], [C190–193][C176], [C250–255][C246].
- Reference sampling coordinates are integers by construction. Manual bilinear code still expresses four texel fetches for them; the compiler may remove some redundant work. A specialized integer reference sampler could preserve exact integer-coordinate values, but must preserve bounds, alpha replacement, and mask resampling at fractional mask coordinates. [C38][C38], [C213][C213].
- Pose/intrinsic conversions already occur once per reference call, not inside CPU pixel loops. Source projection itself still performs per-point camera dot products, perspective division, distortion, and finiteness checks. A relative-camera transform can eliminate repeated world-camera transforms, but reassociation changes float rounding; lens distortion remains nonlinear. [G1980–1983][G1980], [C164–173][C164].
- Full RGB/mask re-sampling is repeated for overlapping reference patches. Reuse can be across depths/sources and also across neighboring centers. A precomputed per-offset photometric cost field followed by the same patch aggregation is a possible larger redesign; preserve valid-sample denominators, source averaging, boundaries, and accumulation order. [C176][C176].
- Several projection/texture calculations use `precise`. Removing it, enabling fast math, using lower precision, or replacing four fetches by hardware bilinear sampling can alter costs near rejection/tie thresholds. These are accuracy-affecting experiments, not guaranteed equivalent speedups. [C78–130][C76], [C164–173][C164].

Because references are processed serially with decode/setup before submission and candidate generation/saving afterward, the GPU has no next-reference work queued by this routine during these CPU phases. The existing fence is necessary for current readback consumers; simply deleting it would introduce races. Resource reuse and a bounded multi-reference pipeline would be the appropriate design direction after measuring those idle gaps. [M3870][M3870], [G2079][G2079], [M4038][M4038].

## 7. Image caching and CPU fusion scaling

There is no decoded-image cache in `BuildColmapPlaneSweepPointCloud`; reference and source image objects are local to each loop iteration. Each successful role calls WIC decoding again. With R=299 and four available sources, this makes **1,495 successful RGB role loads** plus masks if present; additional attempted source candidates can increase failed/empty-image work. OS file caching may reduce disk reads but does not reuse decoded RGBA vectors. [M3873–3904][M3870], [T232][T232].

One 4032 × 3040 RGBA image is 49,029,120 bytes. Five such images are 245,145,600 bytes/reference, or 73,298,534,400 bytes across 299 references, excluding masks/dummies/alignment. The GPU uploads this data again for each role even if a camera was just used as another reference's source. Caching all 299 RGB images would itself consume about 14.66 GB of pixel payload, so a measured, bounded cache is preferable to assuming unlimited residency. [G1961–1979][G1961], [G2036–2069][G2036].

Cross-view neighbor precomputation is the clearest CPU optimization preserving semantics: construct the exact same full sorted order once per reference map and reuse its first C entries for every point from that map. Preserve input vector order and equal-score behavior, especially at the C cutoff. The sorted lists must be built from **retained, successfully processed maps**, not all camera poses. [M3131–3158][M3131].

Voxel fusion uses unordered-map insertion and sums in candidate order; average/count filtering differs from CUDA Fusibile. The voxel hash combines axis hashes with shifted XOR, which can collide for structured coordinates; record load factor, bucket distribution, and time before redesigning it. Voxel observation count counts candidate points, not distinct cameras. Parallel accumulation or a different container can change float sums and output ordering. [M3094–3099][M3094], [M3224–3261][M3215].

The outlier filter searches all occupants of 27 neighboring cells and continues even after reaching its minimum-neighbor requirement. For keep/reject semantics alone, stopping once enough neighbors have been counted is equivalent, provided all loop levels stop correctly. Dense same-cell populations can otherwise approach quadratic work. This branch is currently disabled. [M3281–3345][M3281].

Correctness detail to preserve during performance work: cross-view initializes `smallestRelativeDifference` to finite `float::max`, then tests `isfinite`. An empty neighborhood can therefore increment `validViews`, although it cannot increment agreements under the ordinary tolerance. With current `validViews >= minimumViews && agreeingViews >= minimumViews`, agreement count still controls acceptance. Do not silently change this behavior inside a performance patch; document it separately. [M3170–3201][M3170].

## 8. Existing timings and historical evidence

| Instrument | What it actually includes | Limitation |
|---|---|---|
| Scene wall timer | GPU object initialization, all reconstruction stages, file output, final CPU display mesh | Excludes preceding camera parsing/startup shader compilation and later display GPU uploads; [S1158–1365][S1156] |
| `m_uploadSetupMilliseconds` | API allocation, staging memcpy, descriptor/pipeline creation and command recording from `gpuSetupStart` | Excludes pre-timer large CPU diagnostic vector initialization; GPU uploads have not executed yet; [G1987–2015][G1987], [G2076][G2076] |
| `m_dispatchMilliseconds` | CPU submit through fence completion | Includes queue delay, upload commands, zero fill, compute, all diagnostic readback transfers, and barriers; not isolated compute; [G2078–2087][G2079] |
| `m_readbackComparisonMilliseconds` in normal mode | Host destination allocation, mapping/memcpy, extra vector copies, accepted/nonfinite scan | Device-to-host copies already finished in dispatch+fence; [G2088–2104][G2088] |
| Total GPU wall time | Whole GPU function, including initial vectors/logging and cleanup | Excludes upstream decoding/source ranking and downstream candidate/save/consistency; [M4017–4034][M4020] |
| Candidate materialization timer | CPU point construction following successful GPU depth | Separate from file writing; [M4041–4046][M4038] |
| Experiment `computePlaneCostMs` | CPU timers around **every** plane-cost call | Can introduce massive timer-call count; not a GPU timing; [M3018–3034][M3018] |
| Experiment cross-view/filter timers | Whole CPU calls | Good stage granularity, but normal Scene path passes null metrics; [M4188–4203][M4188], [S1353][S1351] |

There are no GPU timestamp-query calls in the inspected reconstruction path. The approximately fixed number of ordinary per-stage `steady_clock` reads per reference is unlikely to explain an hour by itself; their overhead remains unmeasured. By contrast, per-plane CPU timing at dense P and D could execute 108,753,715,200 clock reads over 299 all-valid references. That mechanism is disabled by the null metrics pointer in normal operation. It becomes relevant to CPU fallback/compare when experiment metrics are passed. The shader's **diagnostic writes and host readbacks**, not host clock reads, are the verified large instrumentation burden in normal GPU mode.

`UIUtility::Print` calls `OutputDebugStringA`, not durable console/file logging. Per-reference summary formatting and debugger handling should be measured but are far less voluminous than per-pixel logging in normal mode. [UIUtility.cpp:90–93][UI90], [G1955–1960][G1929]. Debug pixel diagnostics and experiment runs are separate paths with extra output and changed workloads.

The normal GPU summary reports attempted/succeeded references, fallback counts/reasons, allocation/dispatch failures, GPU wall, setup, dispatch+fence, host readback and candidate materialization. [M4107–4126][M4107]. These are useful first checks, but their labels have the boundaries above. GPU texture/confidence rejection status counts are not accumulated into `result.m_pointsRejectedTexture`/`Confidence` in the successful GPU path; the final generic counters can consequently look zero despite shader rejection. [M4035–4047][M4038], [G2099–2104][G2099], [M4206–4211][M4206]. Obtain rejection fractions from GPU status data rather than those generic counters.

### Existing local experiment artifacts, not new benchmarks

The ignored folder `bin/x64/Debug/Output/PlaneSweep/Experiments` contains JSON measurements:

| Artifact | Total ms | CPU BuildReferencePoints ms | CPU ComputePlaneCost ms | Cross-view ms | References / source role loads |
|---|---:|---:|---:|---:|---:|
| [baseline_1790268305/metrics.json][oldbaseline] | 1,025,500 | 976,224 | 971,638 | 0.0008 | 30 / 180 |
| [PointStep_1_1790286198/metrics.json][oldstep] | 3,885,380 | 3,836,430 | 3,822,990 | 0.0013 | 30 / 180 |
| [ImageScale_50_1790275034/metrics.json][oldscale] | 3,905,480 | 3,856,400 | 3,838,680 | 0.0016 | 30 / 180 |
| [CrossView_On_1790287503/metrics.json][oldcross] | 1,014,170 | 962,753 | 958,269 | 3,010 | 30 / 180 |

These artifacts show historical runs in which the recorded CPU cost function accounted for about 94.75% of baseline wall time and 98.39% of `PointStep_1` wall time. The latter totals about 64.76 minutes, but **must not be identified as the user's one-hour GPU run**: it processes only 30 references with six loaded sources per reference and lacks execution-mode, exact settings, binary, shader and hardware provenance. Current GPU mode skips CPU BuildReferencePoints on success. These files demonstrate why fallback/build/mode provenance matters; they do not measure current GPU bottlenecks. JSON records are preferable to the neighboring `results.csv`, whose displayed rows do not consistently align with its extra `group,value` header columns.

## 9. Comparison with CVP-MVSNet

CVP-MVSNet's released `network.forward` extracts reference/source feature pyramids, builds a coarse feature cost volume, aggregates by variance, regularizes with a 3D network, and regresses depth from probabilities. It upsamples and refines through finer levels. Confidence derives from probability mass around the regressed index. This differs from RGB MAD and a best-versus-second cost gap. [Official net.py, lines 89–160](https://github.com/JiayuYANG/CVP-MVSNet/blob/master/CVP_MVSNet/models/net.py#L89-L160).

Its modules set 48 initial hypotheses and `d=4` for eight refinement hypotheses; warping builds a pixel grid and applies relative camera transforms in batches before bilinear feature sampling. Feature extraction replaces an explicit repeated raw-image patch cost. The released network still extracts source features per forward call, so this report does not claim cross-reference feature caching there. [Official modules.py](https://github.com/JiayuYANG/CVP-MVSNet/blob/master/CVP_MVSNet/models/modules.py). The paper motivates restricting finer-stage search around the current estimate. [CVPR paper](https://openaccess.thecvf.com/content_CVPR_2020/papers/Yang_Cost_Volume_Pyramid_Based_Depth_Inference_for_Multi-View_Stereo_CVPR_2020_paper.pdf).

The released eval script selects **five scales and four source images**. [Official eval.sh:20–25](https://github.com/JiayuYANG/CVP-MVSNet/blob/master/CVP_MVSNet/eval.sh#L20-L25). Assuming those five scales and exact halving to 1/16 width/height, its nominal depth-cell count is:

`P × [48/256 + 8×(1/64 + 1/16 + 1/4 + 1)] = 10.8125P`.

At 1600 × 1184 this is 20,483,200 depth cells/reference, versus 181,862,400 for 96 full-grid planes: about **8.88 times fewer nominal depth cells**. This is an arithmetic comparison, **not a predicted runtime speedup**. If the custom CVP run used a different scale count, recalculate the sum. General form: `Dcoarse × Wcoarse × Hcoarse + 8 × sum(Wfine × Hfine)`. Feature channels, learned convolutions, memory traffic, precision, padding and source aggregation remain additional costs; multiplying the cell ratio by patch area would not yield a defensible timing prediction.

The unavoidable cost of the **current chosen exhaustive algorithm** includes evaluating its configured hypotheses and sources, their projection/sampling, and required acceptance decisions. Recomputing the same reference ray/color for every source/depth, unused diagnostic transfers, per-point neighbor sorting, and per-reference resource creation are implementation costs. A pyramid, fewer hypotheses, resized images, altered patch support, or changed fusion are algorithm changes and require separate accuracy evaluation.

The user-reported totals imply approximately 15× longer than CVP depth estimation alone (60/4) and 4.3× longer than CVP depth estimation plus Fusibile (60/14). Those ratios compare different stage boundaries until plane-sweep compute, CPU consistency, export and display costs are separated. The official CVP instructions invoke Fusibile as a distinct downstream program. [Official fusion instructions](https://github.com/JiayuYANG/CVP-MVSNet#5-generate-point-clouds-and-reproduce-dtu-results). Current plane-sweep voxel averaging and neighborhood depth support are not equivalent to Fusibile, and current voxel/outlier branches are disabled. No numerical or visual equivalence follows from matching image dimensions or source counts.

## 10. Ranked suspected bottlenecks

Ranking is an investigation priority, not measured time attribution.

| Priority | Suspect | Verified evidence | Remaining hypothesis / decisive measurement |
|---|---|---|---|
| 1 | Diagnostic allocation, full readback and CPU copies | Approximately 2.06 GB output payload/reference in the radius-2 comparison case; several copies/allocations | Measure transfer GPU timestamps, allocation/host-copy CPU spans, memory/page faults; determine wall share |
| 2 | Exhaustive shader and repeated rays/reference samples | Up to 18.19 billion patch iterations/reference at 96/4/25 | Isolate compute duration and inspect shader stalls/occupancy; may exceed priority 1 in actual time |
| 3 | CPU cross-view per-point sorting | `Q` sorts of `K-1` fixed camera scores | Time stage plus ranking substage; log Q,K and CPU stacks; could become first with many accepted points |
| 4 | Full CPU fallback or compare mode | Explicit code paths run full scalar search | Check mode, success/fallback counts and reasons; if active, immediately move this to first |
| 5 | Repeated image decode/upload and serial CPU phases | No persistent CPU/GPU image cache; no reference overlap | Time decode/upload, observe GPU idle gaps and source reuse counts |
| 6 | Recreated pipeline and numerous allocations | Every reference creates 18 images and 23 buffers, plus pipeline/descriptors | Time driver calls and resource teardown; inspect memory budget/eviction |
| 7 | CPU candidates, retained maps, PNG output and display mesh | Several O(P)/O(Q) passes and large resident containers | Stage timings and resident/committed memory; data-dependent completeness |
| 8 | Voxel fusion/outlier behavior when enabled | Hash allocation, 27-cell neighbor scans without saturation exit | Current flags disable both; measure only if historical or future run enables them |

The 23 buffers are one image staging buffer, eight input staging/device buffers, and fourteen output/readback buffers. Dummy images are included in the 18 image count. Allocation count does not imply each allocation is individually expensive enough to dominate.

## 11. Minimal profiling plan before optimization

### 11.1 Preserve and identify the baseline

Use the present source/configuration and immutable output location for baseline capture. Record effective parsed settings once, not only INI text; paths, source selection/order, per-reference near/far, input sizes, accepted/rejected counts, loaded reference counts, executable and resolved SPIR-V hashes, build/compiler flags, validation/trace status, device/driver, and available memory. Keep existing `.bin` depths and point outputs from the baseline; add a point export only to a separate instrumentation build if needed. Current code does not export the final point cloud in this path.

First recover a normal GPU summary in a new baseline run; require zero fallback for a GPU-only timing claim. Keep a separate CPU reference and controlled CPU/GPU comparisons for validation. Avoid full dense `CPU_GPU_COMPARE` for routine performance measurements because it intentionally adds CPU search and map comparison. The existing disabled synthetic tests are potential validation assets, not proof of current correctness until rerun. [M3374–3376][M3374], [M3580–3581][M3580].

The first run should preserve existing parameters. Profile representative individual references from that run without changing their per-view search settings; include a boundary/mask-heavy view and a dense foreground view when available. Then validate end-to-end aggregate time. Record cold startup separately from repeated runs; do not silently discard expensive first-use allocation or compilation. Controlled repeat timings should state warm/cold file cache and shader/pipeline state.

### 11.2 CPU timers at stage boundaries

Use `std::chrono::steady_clock` duration scopes or `QueryPerformanceCounter`. No per-pixel/per-plane timer calls. Buffer records in memory and write them once per reference or after the run. Suggested non-overlapping spans:

| Scope | Placement |
|---|---|
| Whole reconstruction, plus separate display mesh | Around `BuildColmapPlaneSweepPointCloud` at [S1353][S1351]; separately around AddColmapPointCloudObject |
| Global preprocessing | Lookups and sparse parse/index at [M3729–3746][M3724] |
| Reference decode and mask decode | Around each `LoadTexture` in [M1756,1768][M1733]; count bytes and image IDs |
| Source ranking and source loading | Separate [M3881][M3870] from [M3893–3904][M3890] |
| Depth range | [M3915–3937][M3915] |
| GPU function preallocation | Before [G1987][G1987] through existing `gpuSetupStart` |
| API/staging/pipeline/recording | Split [G2036–2066][G2036] into coarse allocation, host memcpy, pipeline, command recording spans |
| Submit API and fence wait | Separate calls at [G2080,2083][G2079]; retain existing fence |
| Host readback and teardown | [G2088–2105][G2088], including destruction and vector lifetime overhead |
| Candidate materialization and save | Separate [M4043][M4038] and [M4046][M4038] |
| Cross-view | Whole [M3103][M3103]; optionally aggregate ranking vs depth-check CPU spans on sampled chunks, not each point |
| Voxel/outlier/display | Separate branches in [M3215][M3215] and [P40][P40] |

Per-reference data fields: `reference_id, effective_w,h,point_step,P,planes,sources,patch_area,presearch_passed,accepted,Q_before,Q_after,upload_bytes,readback_bytes,gpu_success,fallback_reason,cpu_stage_ms[],gpu_stage_ms[]`. Status histograms already distinguish reference-invalid/texture-rejected/search results; reduce existing downloaded statuses on the CPU for the first baseline. Do not add a global atomic increment inside every patch loop. Exact projection/sample counts can be collected in a separate sampled diagnostic run if needed, keeping profiling overhead explicit.

### 11.3 Vulkan timestamp queries

Add a small persistent `VK_QUERY_TYPE_TIMESTAMP` pool to an instrumentation-only build. Check the **actual submission queue family's** `timestampValidBits` and `timestampPeriod`; enumerate and log its family index. Reset only slots whose previous submissions have completed. Use four timestamps within the existing production command buffer:

```cpp
vkCmdResetQueryPool(cmd, queries, base, 4);
vkCmdWriteTimestamp(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, queries, base + 0);
// Existing image/input uploads, zero fill, transfer-to-compute barriers.
vkCmdWriteTimestamp(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, queries, base + 1);
vkCmdDispatch(cmd, groupsX, groupsY, 1);
vkCmdWriteTimestamp(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, queries, base + 2);
// Existing compute-to-transfer barriers, seven output copies, host barriers.
vkCmdWriteTimestamp(cmd, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, queries, base + 3);
```

Insertion sites are [G2067–2074][G2067]. Retain all existing barriers and the existing fence. After that fence signals, retrieve 64-bit results with availability via `vkGetQueryPoolResults`; do not add `vkQueueWaitIdle`, a second fence, or an immediate blocking query wait. Pool/slots can be reused on the next completed reference in this synchronous baseline. If later pipelined, use a per-in-flight-slot ring.

Compute elapsed ticks modulo the queue's valid-bit width (special-case 64 to avoid shifting by 64), then `milliseconds = double(deltaTicks) × timestampPeriod × 1e-6`. Interpret q1→q2 as the compute interval, q0→q1 as upload/zero/dependency interval and q2→q3 as readback/dependency tail. GPU timestamps latch at supported stages and intervals may include pipeline stalls or coarser implementation timing; verify boundaries against a trace rather than assuming every interval is perfectly isolated. For a more conservative cross-check, measure the whole submission q0→q3 and, in a separate capture, bracket compute with top/bottom timestamps. Do not insert full-device barriers to force prettier numbers. [Vulkan timestamp specification](https://docs.vulkan.org/spec/latest/chapters/queries.html), [Khronos timestamp sample](https://docs.vulkan.org/samples/latest/samples/api/timestamp_queries/README.html).

Measure instrumentation overhead with otherwise identical builds/runs with query/timer recording on and off. Keep diagnostic buffers enabled for the initial unchanged baseline so the first measurements describe current behavior. Their removal is a later, separately measured candidate.

### 11.4 Capture utilization and memory only where needed

Use a short Vulkan capture in **Nsight Systems** for CPU scheduling, WIC/decode activity, driver allocation, queue gaps, copies, fence waits, memory pressure and GPU metrics. Use **Nsight Graphics GPU Trace** for the production dispatch's throughput/cache/stall behavior and shader inspection supported by the installed version. These tools support Vulkan; CUDA-only kernel profiling is not the correct first tool for this shader. [NVIDIA Nsight Systems](https://developer.nvidia.com/nsight-systems), [NVIDIA Nsight Graphics](https://developer.nvidia.com/nsight-graphics).

Record active warps/occupancy, register/spill evidence where exposed, active-lane efficiency, SM/texture/memory throughput, cache hit rates, PCIe copy intervals, GPU clocks/power/thermal state, host committed/resident memory and page faults. Utilization percentage alone cannot distinguish useful matching from diagnostic transfers. Do not infer memory bandwidth saturation from transfer byte counts alone.

Decision order after the capture: eliminate fallback/mode mistakes; identify longest measured stage; separate shader time from transfers; separate cross-view ranking from consistency comparisons; then choose a candidate below. Report total time, per-reference median/tail, throughput per search-ready center, and output comparisons for every candidate. Use Amdahl's law on measured stage shares before estimating an end-to-end opportunity; no speedup estimate is established here.

## 12. Conditional optimization recommendations

These are proposals for **after baseline profiling**, not implemented changes or measured improvements. Benefit ratings describe the amount of redundant work visible in code and conditions for relevance.

| Order | Candidate | Expected opportunity if confirmed | Difficulty | Correctness / accuracy effects and validation |
|---|---|---|---|---|
| 1 | Separate production depth output from optional diagnostic outputs; keep full diagnostics for validation builds | Very high transfer/allocation reduction: output can approach `4E` depth bytes plus genuinely needed status/counters, instead of ~2.06 GB/reference in example | Medium: separate shader/descriptor contract and host path | No intentional depth-rule change. Keep matching arithmetic unchanged; compare all depth entries, statuses where enabled, candidates and saved previews. Removing writes can affect compiler decisions, so equivalence still must be checked. |
| 2 | Cache cross-view sorted neighbor lists per retained reference map | Very high when Q is large and consistency ranking dominates | Low | Preserve full sort comparator/input order and ties exactly; compare chosen IDs and final surviving point indices bit-for-bit. No intended depth or completeness change. |
| 3 | Retain pipelines, layouts, sampler, descriptor pools, dummy images, reusable buffers; suballocate memory | Medium/high when API/allocation/compilation spans matter | Medium | Same shader and descriptor data; preserve per-reference lifetime/barriers. Compare depth/points exactly; avoid reusing storage still in flight. |
| 4 | Bounded CPU decoded-image cache and GPU resident-image cache | Medium/high if decode/upload or gaps dominate; repeated roles provide reuse | Medium | Cache original bytes and mask interpretation keyed by full path/version/calibration as appropriate; no resizing. Check memory budget and eviction; compare selected source order and outputs. |
| 5 | Remove duplicate depth/status host copies; move result vectors; reserve candidate capacity based on measured counts | Medium for large outputs/Q; low risk | Low | Preserve append order and contents. Do not reserve a worst-case hundreds-of-millions allocation blindly. |
| 6 | Precompute exact reference normalized undistorted coordinates, reference colors/opacity and plane-depth list | High shader opportunity if repeated ALU/fetch work dominates | Medium | A cached `localX,localY` using the same five iterations can preserve later world-point arithmetic. Precomputing a combined world ray or relative transform reassociates operations and may change depths. Compare selected planes, rejection masks and threshold-edge cases. Account for cache bandwidth. |
| 7 | Reference integer-fetch specialization; reuse reference patch across sources/depths; optional shared tiles | Potentially high for repeated reference fetches | Medium/high | Preserve original-coordinate patches, fractional mask sampling and bounds. Large per-thread arrays can spill or reduce occupancy; measure register pressure and runtime before keeping. |
| 8 | Overlap next-reference decode/upload with current GPU work; asynchronously save already completed outputs | Medium/high if timeline has CPU-driven GPU gaps | High | Bounded in-flight resources, correct queue synchronization and deterministic output/candidate order required. Fence removal alone is invalid. Include RAM/VRAM growth in evaluation. |
| 9 | Parallelize CPU point materialization/consistency using fixed output slots and stable compaction | Medium/high only when these measured CPU spans dominate after neighbor caching | Medium | Per-point independent math can stay unchanged. Preserve output order, counts and depth-map snapshots. Avoid parallel unordered floating reductions. |
| 10 | Outlier early stop after sufficient neighbors; improve fusion/hash allocation when enabled | High only in dense outlier cases; currently irrelevant with flags off | Low/medium | Early neighbor saturation preserves keep/reject behavior. Hash changes may reorder fused output; compare point multisets and summed values. |
| 11 | Workgroup-shape tuning or plane/patch-parallel matching and reductions | Unknown until shader profiling identifies occupancy/latency bottleneck | Medium/high | A shape-only change can preserve per-pixel arithmetic. Parallel reductions change addition/tie order unless explicitly stabilized; best/second-best and validity semantics must match. More dispatches are not inherently faster. |
| 12 | Shader optimization flags, hardware bilinear, reduced precision, analytic homographies/undistorted input images | Potential but unmeasured | Medium/high | **Numerical/photometric behavior can change.** Distorted cameras require correct model handling; UNORM sampling and accumulation differences may affect winners/confidence/completeness. Keep as separate accuracy experiments. |
| 13 | Coarse-to-fine adaptive planes, fewer sources/planes, changed image/grid/patch support, new fusion thresholds | Potentially very high algorithmic work reduction | High | **Changes search space and potentially depth/completeness/fusion.** Requires a distinct algorithm baseline, held-out views/scenes and numerical plus visual/geometry evaluation. Do not include in an implementation-only speedup claim. |

Additional small exact-reuse opportunities: sort sparse depths once for both percentiles; cache intrinsic-ID lookup; reuse camera conversion results. These are verified redundancies, but are lower priority than matching, diagnostics and per-point sorting unless measurements show otherwise. [M1939–1940][M1894], [M3357][M3357], [M1320][M1320].

The configured folder has masks matching all 299 registered image filenames. The current implementation rejects a transparent reference center before expensive search using a successfully loaded mask's alpha or luminance when available. Count reference-invalid statuses and document mask decoding, coverage, alpha/luminance interpretation and sampled boundary support before estimating matching work saved. Changing existing masks or thresholds would intentionally change point completeness and matching support. GPU diagnostic allocation/readback currently remains large even for rejected centers. [C226][C226], [M1764–1784][M1761], [G1987][G1987].

With existing masks, an exact reuse candidate is to count valid opaque reference-patch samples once, after the existing texture gate, and skip all planes when fewer than `minimumOpaqueSamples` exist: no source can then supply enough paired samples at any depth. Preserve the existing no-valid-plane result and diagnostic semantics, and validate boundaries. A compact active-center list could improve execution coherence in heavily masked images, but adds compaction/dispatch overhead; profile it separately. Neither is a reason to change masks or thresholds in the baseline.

## 13. Baseline comparison requirements

For every implementation candidate, retain the original executable, source/config hashes, shader bytes, input identities and baseline outputs. Compare dimensions, source order, near/far and plane list; exact zero/nonfinite/positive classifications; selected depth plane and raw float differences; confidence/texture/cost rejection masks; candidate count/order/position/color; consistency survival; voxel observation counts and averaged outputs when enabled. Include contours, thin structures, low texture, occlusions and depth discontinuities in visual review. Do not replace geometric completeness/accuracy analysis with only the count of points.

The existing external CPU/GPU comparison scans all depth entries and identifies closest planes, but it only reports numerical error for positive depths matching the same plane; classification and different-plane differences are separate counters. It caps printed examples at 20. [M3755–3867][M3755]. Keep this distinction when reporting validation. For pure caching/resource-lifetime changes, demand identical outputs where feasible. For floating-point reassociation, set and disclose tolerances **and** count changed plane selections/rejections. For coarse-to-fine or fusion changes, treat any altered depth or completeness as an algorithm effect and evaluate separately.

No performance improvement has been measured or claimed by this audit. The immediate deliverable is a complete source trace, quantified work/memory accounting, and a plan that can determine which verified inefficiencies actually dominate the RTX 4090 run.

## Appendix: compute-shader inventory and dispatch locations

Only the production depth-map entry is called on the ordinary successful GPU reconstruction route. Legacy integration tests are gated off by `runLegacySyntheticSmokeTests=false`; `Initialize` does not invoke the other tests. They may still be compiled by startup shader compilation. One production dispatch runs all depths; there is no production dispatch for CPU consistency, fusion, outliers, or point materialization.

| Shader path under workspace root | C++ entry in `GraphicsFXCore/src/PlaneSweepGpu.cpp` | Dispatch line; role |
|---|---|---|
| `Shaders/planeSweepDepthMapSmokeTest.comp:3,211` | `RunProductionDepthMap:1911`; `RunDepthMapSmokeTest:1732` | 2072 production / 1835 test; 8×8×1 |
| `Shaders/planeSweepTextureStdDevSmokeTest.comp:3,109` | `RunTextureStdDevSmokeTest:1647` | 1680; test |
| `Shaders/planeSweepDepthPlaneSearchSmokeTest.comp:3,117` | `RunDepthPlaneSearchSmokeTest:1511` | 1611; test |
| `Shaders/planeSweepMultiSourceCostSmokeTest.comp:3,154` | `RunMultiSourceCostSmokeTest:1378` | 1489; test |
| `Shaders/planeSweepSingleSourceCostSmokeTest.comp:3,229` | `RunSingleSourceCostSmokeTest:1109` | 1311; test |
| `Shaders/planeSweepProjectPointSmokeTest.comp:3,97` | `RunProjectPointSmokeTest:885` | 1010; test |
| `Shaders/planeSweepUnprojectPixelSmokeTest.comp:3,80` | `RunUnprojectPixelSmokeTest:712` | 834; test |
| `Shaders/planeSweepMultiSourceDescriptorSmokeTest.comp:3,56` | `RunMultiSourceDescriptorSmokeTest:464` | 669; test |
| `Shaders/planeSweepPixelSamplingSmokeTest.comp:3,104` | `RunPlaneSweepPixelSamplingSmokeTest:4060`; `RunRealImageSamplingSmokeTest:4438` | 4341 / 4655; tests |
| `Shaders/planeSweepBilinearSamplingSmokeTest.comp:3,58` | `RunBilinearSamplingSmokeTest:3609` | 3933; test |
| `Shaders/planeSweepImageSamplingSmokeTest.comp:3,16` | `RunImageSamplingSmokeTest:3148` | 3520; test |
| `Shaders/planeSweepCameraLayoutSmokeTest.comp:3,53` | `RunCameraLayoutSmokeTest:2645`; `RunCameraConversionSmokeTest:61` | 3013 / 309; tests |
| `Shaders/planeSweepBufferSmokeTest.comp:3,10` | `RunStorageBufferSmokeTest:2274` | 2542; test |
| `Shaders/planeSweepSmokeTest.comp:3,7` | `RunComputeSmokeTest:2164` | 2237; test |

Workspace root: `C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master`. Public GPU types/API are in [PlaneSweepGpuTypes.h][gputypes] and [PlaneSweepGpu.h][gpuheader]. CPU image/map/candidate structures and helper declarations are in [MultiViewUtility.h:463 onward][MH463].

[manifest]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/Documents/PerformanceAudit/baseline/sha256-manifest.json
[baselineini]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/Documents/PerformanceAudit/baseline/setup.ini
[observations]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/Documents/PerformanceAudit/observations.json
[gpuheader]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/include/PlaneSweepGpu.h
[gputypes]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/include/PlaneSweepGpuTypes.h
[oldbaseline]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/bin/x64/Debug/Output/PlaneSweep/Experiments/baseline_1790268305/metrics.json
[oldstep]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/bin/x64/Debug/Output/PlaneSweep/Experiments/PointStep_1_1790286198/metrics.json
[oldscale]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/bin/x64/Debug/Output/PlaneSweep/Experiments/ImageScale_50_1790275034/metrics.json
[oldcross]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/bin/x64/Debug/Output/PlaneSweep/Experiments/CrossView_On_1790287503/metrics.json
[S129]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsApp00/src/Scene.cpp:129
[S171]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsApp00/src/Scene.cpp:171
[S1156]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsApp00/src/Scene.cpp:1156
[S1173]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsApp00/src/Scene.cpp:1173
[S1347]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsApp00/src/Scene.cpp:1347
[S1351]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsApp00/src/Scene.cpp:1351
[S1428]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsApp00/src/Scene.cpp:1428
[M589]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:589
[M1320]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:1320
[M1391]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:1391
[M1733]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:1733
[M1761]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:1761
[M1818]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:1818
[M1894]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:1894
[M2063]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:2063
[M2236]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:2236
[M2907]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:2907
[M2950]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:2950
[M2984]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:2984
[M3018]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3018
[M3094]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3094
[M3103]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3103
[M3131]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3131
[M3170]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3170
[M3215]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3215
[M3281]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3281
[M3357]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3357
[M3374]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3374
[M3580]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3580
[M3724]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3724
[M3748]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3748
[M3755]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3755
[M3870]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3870
[M3890]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3890
[M3915]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3915
[M3940]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3940
[M3955]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3955
[M3970]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3970
[M3992]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:3992
[M4020]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:4020
[M4038]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:4038
[M4101]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:4101
[M4107]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:4107
[M4188]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:4188
[M4206]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.cpp:4206
[MH463]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.h:463
[MH497]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/MultiViewUtility.h:497
[G391]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:391
[G443]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:443
[G1682]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:1682
[G1929]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:1929
[G1961]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:1961
[G1980]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:1980
[G1987]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:1987
[G2003]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:2003
[G2018]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:2018
[G2036]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:2036
[G2057]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:2057
[G2059]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:2059
[G2060]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:2060
[G2065]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:2065
[G2067]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:2067
[G2071]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:2071
[G2072]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:2072
[G2073]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:2073
[G2076]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:2076
[G2079]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:2079
[G2088]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:2088
[G2099]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:2099
[G2105]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/PlaneSweepGpu.cpp:2105
[C3]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/Shaders/planeSweepDepthMapSmokeTest.comp:3
[C24]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/Shaders/planeSweepDepthMapSmokeTest.comp:24
[C38]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/Shaders/planeSweepDepthMapSmokeTest.comp:38
[C48]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/Shaders/planeSweepDepthMapSmokeTest.comp:48
[C76]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/Shaders/planeSweepDepthMapSmokeTest.comp:76
[C134]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/Shaders/planeSweepDepthMapSmokeTest.comp:134
[C151]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/Shaders/planeSweepDepthMapSmokeTest.comp:151
[C164]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/Shaders/planeSweepDepthMapSmokeTest.comp:164
[C176]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/Shaders/planeSweepDepthMapSmokeTest.comp:176
[C213]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/Shaders/planeSweepDepthMapSmokeTest.comp:213
[C226]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/Shaders/planeSweepDepthMapSmokeTest.comp:226
[C246]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/Shaders/planeSweepDepthMapSmokeTest.comp:246
[C272]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/Shaders/planeSweepDepthMapSmokeTest.comp:272
[C283]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/Shaders/planeSweepDepthMapSmokeTest.comp:283
[U205]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/SetupUtility.cpp:205
[H25]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/ShaderUtility.cpp:25
[H350]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/ShaderUtility.cpp:350
[E274]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/EngineCore.cpp:274
[T232]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/TextureUtility.cpp:232
[V246]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/GraphicsUtility.h:246
[P40]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/SceneObjectPointCloud.h:40
[O68]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/SceneObject.cpp:68
[O237]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/SceneObject.cpp:237
[O1045]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/SceneObject.cpp:1045
[O1463]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/SceneObject.cpp:1463
[O1643]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/SceneObject.cpp:1643
[UI90]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/UIUtility.cpp:90
[I21]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsApp00/setup/setup.ini:21
[I99]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsApp00/setup/setup.ini:99
[I167]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsApp00/setup/setup.ini:167
[I217]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsApp00/setup/setup.ini:217
[I228]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsApp00/setup/setup.ini:228
[I238]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsApp00/setup/setup.ini:238
[I242]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsApp00/setup/setup.ini:242
[I249]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsApp00/setup/setup.ini:249
[I254]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsApp00/setup/setup.ini:254
[I258]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsApp00/setup/setup.ini:258
[I264]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsApp00/setup/setup.ini:264
[W30]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsApp00/src/WinMain.cpp:30
[U168]: C:/Users/pdeshpa1/Documents/GraphicsApp-master/GraphicsApp-master/GraphicsFXCore/src/SetupUtility.cpp:168
