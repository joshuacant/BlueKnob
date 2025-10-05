idf_component_register(
  SRCS "audio_bsp.c"
  PRIV_REQUIRES driver main
  INCLUDE_DIRS "./"
  EMBED_FILES "canon.pcm")
