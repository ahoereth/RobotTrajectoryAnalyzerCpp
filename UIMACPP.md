# UIMACPP

## Quick Links
* [Class List](http://uima.apache.org/d/uimacpp-2.4.0/docs/html/annotated.html)
* [List of error id constants](http://uima.apache.org/d/uimacpp-2.4.0/docs/html/err__ids_8h.html)
* [UIMAJ Reference](https://uima.apache.org/d/uimaj-2.4.0/references.html)


## Shortcomings in contrast to UIMAJ
UIMACPP..
* ..does not have an annotation class which supports `setBegin` and `setEnd` as UIMAJ. Contrary to UIMACPP, UIMAJ's [AnnotationFS](http://uima.apache.org/d/uimaj-2.4.2/apidocs/org/apache/uima/cas/text/AnnotationFS.html) itself is just an interface (which is also missing those functions) - the [Annotation](http://uima.apache.org/d/uimaj-2.4.2/apidocs/org/apache/uima/jcas/tcas/Annotation.html) class is an implementation of AnnotationFS introducing them. **Workaround:** *[Create](http://uima.apache.org/d/uimacpp-2.4.0/docs/html/group__UtilFuncts.html#ga0) new [AnnotationFS](http://uima.apache.org/d/uimacpp-2.4.0/docs/html/classuima_1_1AnnotationFS.html) when the begin and/or end positions change. Future: Create own Annotation class which extends the AnnotationFS with the required functionality.*
* ...does not support configuration parameters of type double (missing extract function in [AnnotatorContext class](http://uima.apache.org/d/uimacpp-2.4.0/docs/html/classuima_1_1AnnotatorContext.html)). **Workaround:** *Use  [floats](http://uima.apache.org/d/uimacpp-2.4.0/docs/html/classuima_1_1AnnotatorContext.html#uima_1_1AnnotatorContextz14_6).*
* ...does not have an equivalent of the [uimaFIT JCasUtil](http://uima.apache.org/d/uimafit-2.0.0/api/org/apache/uima/fit/util/JCasUtil.html) which extends the UIMAJ framework with many convenience functions like [selectCovered](http://uima.apache.org/d/uimafit-2.0.0/api/org/apache/uima/fit/util/JCasUtil.html#selectCovered(java.lang.Class, org.apache.uima.cas.text.AnnotationFS)). **Workaround:** *[Implement them yourself](https://github.com/ahoereth/RobotTrajectoryAnalyzerCpp/blob/master/src/uimacppext/src/utils.cpp).*
