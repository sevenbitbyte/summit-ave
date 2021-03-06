<a name="0.6.1"></a>
## [0.6.1](https://github.com/yoannmoinet/nipplejs/compare/v0.6.1...v0.6.1) (2015-12-26)




<a name="0.6.1"></a>
## [0.6.1](https://github.com/yoannmoinet/nipplejs/compare/v0.6.0...v0.6.1) (2015-12-26)


### Bug Fixes

* **collection:** trigger `start` event before any potential direction ([c5441fd](https://github.com/yoannmoinet/nipplejs/commit/c5441fd))
* **nipple:** fragment control of previous directions ([790fa26](https://github.com/yoannmoinet/nipplejs/commit/790fa26)), closes [#25](https://github.com/yoannmoinet/nipplejs/issues/25)

### Features

* **nipple:** add `resetDirection()` to cancel previous directions ([66f6e3b](https://github.com/yoannmoinet/nipplejs/commit/66f6e3b))



<a name="0.6.0"></a>
# [0.6.0](https://github.com/yoannmoinet/nipplejs/compare/v0.5.6...v0.6.0) (2015-11-29)


### Bug Fixes

* **nipple:** call off() after triggering ([bf8e5c3](https://github.com/yoannmoinet/nipplejs/commit/bf8e5c3))
* **nipple:** only trigger itself, not its manager ([6b39786](https://github.com/yoannmoinet/nipplejs/commit/6b39786))
* **super:** move internal declarations where needed ([a4c79c2](https://github.com/yoannmoinet/nipplejs/commit/a4c79c2))
* **utils:** return object on u.extend ([9197c00](https://github.com/yoannmoinet/nipplejs/commit/9197c00))
* clean identifier if not found in any collection ([3496395](https://github.com/yoannmoinet/nipplejs/commit/3496395))
* handle scrolling offset by resetting zone's box ([6ccd0b5](https://github.com/yoannmoinet/nipplejs/commit/6ccd0b5))
* remove useless injections ([a0b1d7e](https://github.com/yoannmoinet/nipplejs/commit/a0b1d7e))
* return the found nipple in `get()` ([439aa69](https://github.com/yoannmoinet/nipplejs/commit/439aa69))
* unbind document when collection is manually destroyed ([6e66b62](https://github.com/yoannmoinet/nipplejs/commit/6e66b62))

### Features

* **nipple:** add an incremental id to absolutely differentiate each ([598a5eb](https://github.com/yoannmoinet/nipplejs/commit/598a5eb))
* **utils:** add u.map to execute fn in array or single element ([515aa5d](https://github.com/yoannmoinet/nipplejs/commit/515aa5d))
* **utils:** add u.safeExtend to only replace existent attributes ([5212ffe](https://github.com/yoannmoinet/nipplejs/commit/5212ffe))

### Performance Improvements

* safety clean if nipple isn't found in collection ([ca20985](https://github.com/yoannmoinet/nipplejs/commit/ca20985))



<a name="0.5.6"></a>
## [0.5.6](https://github.com/yoannmoinet/nipplejs/compare/v0.5.5...v0.5.6) (2015-11-09)




<a name="0.5.5"></a>
## [0.5.5](https://github.com/yoannmoinet/nipplejs/compare/v0.5.4...v0.5.5) (2015-11-09)


### Bug Fixes

* **manager:** allow multitouch to be more than 1 ([7569536](https://github.com/yoannmoinet/nipplejs/commit/7569536))
* **manager:** clean the start process, no duplicate ([7e30551](https://github.com/yoannmoinet/nipplejs/commit/7e30551))
* **manager:** remove handler only if last touch ([1f03064](https://github.com/yoannmoinet/nipplejs/commit/1f03064))



<a name="0.5.4"></a>
## [0.5.4](https://github.com/yoannmoinet/nipplejs/compare/v0.5.3...v0.5.4) (2015-11-08)


### Bug Fixes

* **manager:** better handle identifiers ([54b3cff](https://github.com/yoannmoinet/nipplejs/commit/54b3cff)), closes [#16](https://github.com/yoannmoinet/nipplejs/issues/16)
* **nipple:** use `document.body.contains` for IE11 doesn't shortcut ([6e5b80a](https://github.com/yoannmoinet/nipplejs/commit/6e5b80a)), closes [#17](https://github.com/yoannmoinet/nipplejs/issues/17)



<a name="0.5.3"></a>
## [0.5.3](https://github.com/yoannmoinet/nipplejs/compare/v0.5.2...v0.5.3) (2015-10-04)


### Bug Fixes

* **manager:** add back pressure variable... oups ([a46982f](https://github.com/yoannmoinet/nipplejs/commit/a46982f))



<a name="0.5.2"></a>
## [0.5.2](https://github.com/yoannmoinet/nipplejs/compare/v0.5.1...v0.5.2) (2015-10-04)


### Bug Fixes

* **manager:** correctly unbind all events when destroying manager ([e3f96ef](https://github.com/yoannmoinet/nipplejs/commit/e3f96ef))
* **utils:** handle touch based on their event's type ([406a7b2](https://github.com/yoannmoinet/nipplejs/commit/406a7b2))

### Features

* **manager:** better pressure management cross-plateform ([d867812](https://github.com/yoannmoinet/nipplejs/commit/d867812))



<a name="0.5.1"></a>
## [0.5.1](https://github.com/yoannmoinet/nipplejs/compare/v0.5.0...v0.5.1) (2015-10-03)


### Bug Fixes

* **nipple:** remove useless backPosition ([4c76adc](https://github.com/yoannmoinet/nipplejs/commit/4c76adc))
* **super:** better inheritance ([4a3a8dc](https://github.com/yoannmoinet/nipplejs/commit/4a3a8dc))



<a name="0.5.0"></a>
# [0.5.0](https://github.com/yoannmoinet/nipplejs/compare/v0.4.2...v0.5.0) (2015-09-29)


### Bug Fixes

* remove useless rimraf ([26db70f](https://github.com/yoannmoinet/nipplejs/commit/26db70f))
* **manager:** add default pressure at 0 ([1cb31d0](https://github.com/yoannmoinet/nipplejs/commit/1cb31d0))
* **nipple:** control dom presence before add or remove ([7cc1d8c](https://github.com/yoannmoinet/nipplejs/commit/7cc1d8c))
* **nipple:** return the manager's known object in event triggers ([2deb42c](https://github.com/yoannmoinet/nipplejs/commit/2deb42c))
* **npm:** add rimraf as a dev dependency ([deb7acc](https://github.com/yoannmoinet/nipplejs/commit/deb7acc)), closes [#12](https://github.com/yoannmoinet/nipplejs/issues/12)

### Features

* **manager:** add destroy method ([295654f](https://github.com/yoannmoinet/nipplejs/commit/295654f))
* **manager:** expose nippleOptions ([968103c](https://github.com/yoannmoinet/nipplejs/commit/968103c))
* add dataOnly option ([2f527b1](https://github.com/yoannmoinet/nipplejs/commit/2f527b1))
* add two modes, static and semi ([347feaa](https://github.com/yoannmoinet/nipplejs/commit/347feaa))
* **manager:** handle resize and reinit positions ([261fb59](https://github.com/yoannmoinet/nipplejs/commit/261fb59))
* **nipple:** add destroy method ([713e505](https://github.com/yoannmoinet/nipplejs/commit/713e505))
* **nipple:** remove position offset ([be59bc4](https://github.com/yoannmoinet/nipplejs/commit/be59bc4))
* **off:** allow unsubscribing on all events at once ([c30bfad](https://github.com/yoannmoinet/nipplejs/commit/c30bfad))



<a name="0.4.2"></a>
## [0.4.2](https://github.com/yoannmoinet/nipplejs/compare/v0.4.1...v0.4.2) (2015-09-15)


### Bug Fixes

* correctly return directions ([871d782](https://github.com/yoannmoinet/nipplejs/commit/871d782))



<a name="0.4.1"></a>
## [0.4.1](https://github.com/yoannmoinet/nipplejs/compare/v0.4.0...v0.4.1) (2015-09-15)


### Bug Fixes

* return the computed direction ([b3d80b3](https://github.com/yoannmoinet/nipplejs/commit/b3d80b3))



<a name="0.4.0"></a>
# [0.4.0](https://github.com/yoannmoinet/nipplejs/compare/v0.3.1...v0.4.0) (2015-09-15)


### Bug Fixes

* **manager:** streamline events triggered ([76941ec](https://github.com/yoannmoinet/nipplejs/commit/76941ec))

### Features

* add more events for both nipple and manager ([f7966b9](https://github.com/yoannmoinet/nipplejs/commit/f7966b9))
* add support for both mouse and touch at the same time ([e8fcba2](https://github.com/yoannmoinet/nipplejs/commit/e8fcba2))
* add support for pressure ([80e7770](https://github.com/yoannmoinet/nipplejs/commit/80e7770)), closes [#10](https://github.com/yoannmoinet/nipplejs/issues/10)
* attach values to each instance of nipple ([c5bf9ab](https://github.com/yoannmoinet/nipplejs/commit/c5bf9ab))
* limit the number of simultaneous nipples ([801e91d](https://github.com/yoannmoinet/nipplejs/commit/801e91d))
* support multitouch correctly ([33bbe23](https://github.com/yoannmoinet/nipplejs/commit/33bbe23)), closes [#2](https://github.com/yoannmoinet/nipplejs/issues/2) [#6](https://github.com/yoannmoinet/nipplejs/issues/6)
* **nipple:** add a callback after show and hide ([5b24124](https://github.com/yoannmoinet/nipplejs/commit/5b24124))
* **nipple:** add the positions to the instance ([fac981b](https://github.com/yoannmoinet/nipplejs/commit/fac981b))
* **off:** allow unsubscribing of a type at once ([cbbefca](https://github.com/yoannmoinet/nipplejs/commit/cbbefca))
* **on:** pass the target in the event ([bc0630d](https://github.com/yoannmoinet/nipplejs/commit/bc0630d))



<a name="0.3.1"></a>
## [0.3.1](https://github.com/yoannmoinet/nipplejs/compare/v0.3.0...v0.3.1) (2015-09-07)


### Bug Fixes

* **on:** allows multiple spaces in event string ([b89bd0e](https://github.com/yoannmoinet/nipplejs/commit/b89bd0e))
* **on:** removes variable name clash. ([3d70a3c](https://github.com/yoannmoinet/nipplejs/commit/3d70a3c))



<a name="0.3.0"></a>
# [0.3.0](https://github.com/yoannmoinet/nipplejs/compare/v0.2.1...v0.3.0) (2015-09-06)


### Features

* offset angle by 180 for units circle ([3002014](https://github.com/yoannmoinet/nipplejs/commit/3002014))
* support IE PointerEvents ([ed65d1e](https://github.com/yoannmoinet/nipplejs/commit/ed65d1e)), closes [#1](https://github.com/yoannmoinet/nipplejs/issues/1)



<a name="0.2.1"></a>
## [0.2.1](https://github.com/yoannmoinet/nipplejs/compare/v0.2.0...v0.2.1) (2015-09-06)


### Bug Fixes

* correct chrome's clientBoundingBox ([b94bc43](https://github.com/yoannmoinet/nipplejs/commit/b94bc43))



<a name="0.2.0"></a>
# [0.2.0](https://github.com/yoannmoinet/nipplejs/compare/v0.1.1...v0.2.0) (2015-09-06)


### Bug Fixes

* **on:** return self to allow chain ([f068cc9](https://github.com/yoannmoinet/nipplejs/commit/f068cc9))

### Features

* **on:** let listen to multiple events ([d19ad97](https://github.com/yoannmoinet/nipplejs/commit/d19ad97))
* **on, off:** allow to chain calls ([c82c580](https://github.com/yoannmoinet/nipplejs/commit/c82c580))
* **trigger:** trigger only if different direction ([9e334ba](https://github.com/yoannmoinet/nipplejs/commit/9e334ba))



<a name="0.1.1"></a>
## [0.1.1](https://github.com/yoannmoinet/nipplejs/compare/v0.1.0...v0.1.1) (2015-09-06)


### Bug Fixes

* add scroll offset ([7b36fb6](https://github.com/yoannmoinet/nipplejs/commit/7b36fb6))
* remove useless log ([b958cbf](https://github.com/yoannmoinet/nipplejs/commit/b958cbf))
* send data to start event ([c1f1688](https://github.com/yoannmoinet/nipplejs/commit/c1f1688))



<a name="0.1.0"></a>
# 0.1.0 (2015-09-05)




