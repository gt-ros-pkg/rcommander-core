#include <Carbon/Carbon.r>

#define Reserved8   reserved, reserved, reserved, reserved, reserved, reserved, reserved, reserved
#define Reserved12  Reserved8, reserved, reserved, reserved, reserved
#define Reserved13  Reserved12, reserved
#define dp_none__   noParams, "", directParamOptional, singleItem, notEnumerated, Reserved13
#define reply_none__   noReply, "", replyOptional, singleItem, notEnumerated, Reserved13
#define synonym_verb__ reply_none__, dp_none__, { }
#define plural__    "", {"", kAESpecialClassProperties, cType, "", reserved, singleItem, notEnumerated, readOnly, Reserved8, noApostrophe, notFeminine, notMasculine, plural}, {}

resource 'aete' (0, "Dictionary") {
	0x1,  // major version
	0x0,  // minor version
	english,
	roman,
	{
		"Standard Suite",
		"Common classes and commands for most applications.",
		'????',
		1,
		1,
		{
			/* Events */

			"open",
			"Open an object.",
			'aevt', 'odoc',
			reply_none__,
			'file',
			"The file(s) to be opened.",
			directParamRequired,
			singleItem, notEnumerated, Reserved13,
			{

			},

			"print",
			"Print an object.",
			'aevt', 'pdoc',
			reply_none__,
			'file',
			"The file(s) or document(s) to be printed.",
			directParamRequired,
			singleItem, notEnumerated, Reserved13,
			{

			},

			"quit",
			"Quit an application.",
			'aevt', 'quit',
			reply_none__,
			dp_none__,
			{
				"saving", 'savo', 'savo',
				"Specifies whether changes should be saved before quitting.",
				optional,
				singleItem, enumerated, Reserved13
			},

			"close",
			"Close an object.",
			'core', 'clos',
			reply_none__,
			'obj ',
			"the object to close",
			directParamRequired,
			singleItem, notEnumerated, Reserved13,
			{
				"saving", 'savo', 'savo',
				"Specifies whether changes should be saved before closing.",
				optional,
				singleItem, enumerated, Reserved13,
				"saving in", 'kfil', 'file',
				"The file in which to save the object.",
				optional,
				singleItem, notEnumerated, Reserved13
			},

			"count",
			"Return the number of elements of a particular class within an object.",
			'core', 'cnte',
			'long',
			"the number of elements",
			replyRequired, singleItem, notEnumerated, Reserved13,
			'obj ',
			"the object whose elements are to be counted",
			directParamRequired,
			singleItem, notEnumerated, Reserved13,
			{
				"each", 'kocl', 'type',
				"The class of objects to be counted.",
				optional,
				singleItem, notEnumerated, Reserved13
			},

			"delete",
			"Delete an object.",
			'core', 'delo',
			reply_none__,
			'obj ',
			"the object to delete",
			directParamRequired,
			singleItem, notEnumerated, Reserved13,
			{

			},

			"duplicate",
			"Copy object(s) and put the copies at a new location.",
			'core', 'clon',
			reply_none__,
			'obj ',
			"the object(s) to duplicate",
			directParamRequired,
			singleItem, notEnumerated, Reserved13,
			{
				"to", 'insh', 'insl',
				"The location for the new object(s).",
				required,
				singleItem, notEnumerated, Reserved13,
				"with properties", 'prdt', 'reco',
				"Properties to be set in the new duplicated object(s).",
				optional,
				singleItem, notEnumerated, Reserved13
			},

			"exists",
			"Verify if an object exists.",
			'core', 'doex',
			'bool',
			"true if it exists, false if not",
			replyRequired, singleItem, notEnumerated, Reserved13,
			'obj ',
			"the object in question",
			directParamRequired,
			singleItem, notEnumerated, Reserved13,
			{

			},

			"get",
			"Get the data for an object.",
			'core', 'getd',
			'****',
			"",
			replyRequired, singleItem, notEnumerated, Reserved13,
			'obj ',
			"",
			directParamRequired,
			singleItem, notEnumerated, Reserved13,
			{

			},

			"make",
			"Make a new object.",
			'core', 'crel',
			'obj ',
			"to the new object",
			replyRequired, singleItem, notEnumerated, Reserved13,
			dp_none__,
			{
				"new", 'kocl', 'type',
				"The class of the new object.",
				required,
				singleItem, notEnumerated, Reserved13,
				"at", 'insh', 'insl',
				"The location at which to insert the object.",
				optional,
				singleItem, notEnumerated, Reserved13,
				"with data", 'data', '****',
				"The initial data for the object.",
				optional,
				singleItem, notEnumerated, Reserved13,
				"with properties", 'prdt', 'reco',
				"The initial values for properties of the object.",
				optional,
				singleItem, notEnumerated, Reserved13
			},

			"move",
			"Move object(s) to a new location.",
			'core', 'move',
			reply_none__,
			'obj ',
			"the object(s) to move",
			directParamRequired,
			singleItem, notEnumerated, Reserved13,
			{
				"to", 'insh', 'insl',
				"The new location for the object(s).",
				required,
				singleItem, notEnumerated, Reserved13
			},

			"save",
			"Save an object.",
			'core', 'save',
			reply_none__,
			'obj ',
			"the object to save, usually a document or window",
			directParamRequired,
			singleItem, notEnumerated, Reserved13,
			{
				"in", 'kfil', 'file',
				"The file in which to save the object.",
				optional,
				singleItem, notEnumerated, Reserved13,
				"as", 'fltp', 'ctxt',
				"The file type in which to save the data.",
				optional,
				singleItem, notEnumerated, Reserved13
			},

			"set",
			"Set an object's data.",
			'core', 'setd',
			reply_none__,
			'obj ',
			"",
			directParamRequired,
			singleItem, notEnumerated, Reserved13,
			{
				"to", 'data', '****',
				"The new value.",
				required,
				singleItem, notEnumerated, Reserved13
			}
		},
		{
			/* Classes */

			"item", 'cobj',
			"A scriptable object.",
			{
				"class", 'pcls', 'type',
				"The class of the object.",
				reserved, singleItem, notEnumerated, readOnly, Reserved12,

				"properties", 'pALL', 'reco',
				"All of the object's properties.",
				reserved, singleItem, notEnumerated, readWrite, Reserved12
			},
			{
			},
			"items", 'cobj', plural__,

			"application", 'capp',
			"An application's top level scripting object.",
			{
				"name", 'pnam', 'ctxt',
				"The name of the application.",
				reserved, singleItem, notEnumerated, readOnly, Reserved12,

				"frontmost", 'pisf', 'bool',
				"Is this the frontmost (active) application?",
				reserved, singleItem, notEnumerated, readOnly, Reserved12,

				"version", 'vers', 'ctxt',
				"The version of the application.",
				reserved, singleItem, notEnumerated, readOnly, Reserved12
			},
			{
				'docu', { },
				'cwin', { }
			},
			"applications", 'capp', plural__,

			"color", 'colr',
			"A color.",
			{
			},
			{
			},
			"colors", 'colr', plural__,

			"document", 'docu',
			"A document.",
			{
				"path", 'ppth', 'ctxt',
				"The document's path.",
				reserved, singleItem, notEnumerated, readWrite, Reserved12,

				"modified", 'imod', 'bool',
				"Has the document been modified since the last save?",
				reserved, singleItem, notEnumerated, readOnly, Reserved12,

				"name", 'pnam', 'ctxt',
				"The document's name.",
				reserved, singleItem, notEnumerated, readWrite, Reserved12
			},
			{
			},
			"documents", 'docu', plural__,

			"window", 'cwin',
			"A window.",
			{
				"name", 'pnam', 'ctxt',
				"The full title of the window.",
				reserved, singleItem, notEnumerated, readWrite, Reserved12,

				"id", 'ID  ', 'nmbr',
				"The unique identifier of the window.",
				reserved, singleItem, notEnumerated, readOnly, Reserved12,

				"bounds", 'pbnd', 'qdrt',
				"The bounding rectangle of the window.",
				reserved, singleItem, notEnumerated, readWrite, Reserved12,

				"document", 'docu', 'docu',
				"The document whose contents are being displayed in the window.",
				reserved, singleItem, notEnumerated, readOnly, Reserved12,

				"closeable", 'hclb', 'bool',
				"Whether the window has a close box.",
				reserved, singleItem, notEnumerated, readOnly, Reserved12,

				"titled", 'ptit', 'bool',
				"Whether the window has a title bar.",
				reserved, singleItem, notEnumerated, readOnly, Reserved12,

				"index", 'pidx', 'nmbr',
				"The index of the window in the back-to-front window ordering.",
				reserved, singleItem, notEnumerated, readWrite, Reserved12,

				"floating", 'isfl', 'bool',
				"Whether the window floats.",
				reserved, singleItem, notEnumerated, readOnly, Reserved12,

				"miniaturizable", 'ismn', 'bool',
				"Whether the window can be miniaturized.",
				reserved, singleItem, notEnumerated, readOnly, Reserved12,

				"miniaturized", 'pmnd', 'bool',
				"Whether the window is currently miniaturized.",
				reserved, singleItem, notEnumerated, readWrite, Reserved12,

				"modal", 'pmod', 'bool',
				"Whether the window is the application's current modal window.",
				reserved, singleItem, notEnumerated, readOnly, Reserved12,

				"resizable", 'prsz', 'bool',
				"Whether the window can be resized.",
				reserved, singleItem, notEnumerated, readOnly, Reserved12,

				"visible", 'pvis', 'bool',
				"Whether the window is currently visible.",
				reserved, singleItem, notEnumerated, readWrite, Reserved12,

				"zoomable", 'iszm', 'bool',
				"Whether the window can be zoomed.",
				reserved, singleItem, notEnumerated, readOnly, Reserved12,

				"zoomed", 'pzum', 'bool',
				"Whether the window is currently zoomed.",
				reserved, singleItem, notEnumerated, readWrite, Reserved12
			},
			{
			},
			"windows", 'cwin', plural__
		},
		{
			/* Comparisons */
		},
		{
			/* Enumerations */
			'savo',
			{
				"yes", 'yes ', "Save the file.",
				"no", 'no  ', "Do not save the file.",
				"ask", 'ask ', "Ask the user whether or not to save the file."
			}
		},

		"Text Suite",
		"A set of basic classes for text processing.",
		'????',
		1,
		1,
		{
			/* Events */

		},
		{
			/* Classes */

			"text", 'ctxt',
			"Rich (styled) text",
			{
				"color", 'colr', 'colr',
				"The color of the first character.",
				reserved, singleItem, notEnumerated, readWrite, Reserved12,

				"font", 'font', 'ctxt',
				"The name of the font of the first character.",
				reserved, singleItem, notEnumerated, readWrite, Reserved12,

				"size", 'ptsz', 'nmbr',
				"The size in points of the first character.",
				reserved, singleItem, notEnumerated, readWrite, Reserved12
			},
			{
				'cpar', { },
				'cwor', { },
				'cha ', { },
				'catr', { },
				'atts', { }
			},
			"text", 'ctxt', plural__,

			"attachment", 'atts',
			"Represents an inline text attachment.  This class is used mainly for make commands.",
			{
				"<Inheritance>", pInherits, 'ctxt',
				"inherits elements and properties of the text class.",
				reserved, singleItem, notEnumerated, readOnly, Reserved12,

				"file name", 'atfn', 'ctxt',
				"The path to the file for the attachment",
				reserved, singleItem, notEnumerated, readWrite, Reserved12
			},
			{
			},
			"attachments", 'atts', plural__,

			"paragraph", 'cpar',
			"This subdivides the text into paragraphs.",
			{
				"<Inheritance>", pInherits, 'ctxt',
				"inherits elements and properties of the text class.",
				reserved, singleItem, notEnumerated, readOnly, Reserved12
			},
			{
			},
			"paragraphs", 'cpar', plural__,

			"word", 'cwor',
			"This subdivides the text into words.",
			{
				"<Inheritance>", pInherits, 'ctxt',
				"inherits elements and properties of the text class.",
				reserved, singleItem, notEnumerated, readOnly, Reserved12
			},
			{
			},
			"words", 'cwor', plural__,

			"character", 'cha ',
			"This subdivides the text into characters.",
			{
				"<Inheritance>", pInherits, 'ctxt',
				"inherits elements and properties of the text class.",
				reserved, singleItem, notEnumerated, readOnly, Reserved12
			},
			{
			},
			"characters", 'cha ', plural__,

			"attribute run", 'catr',
			"This subdivides the text into chunks that all have the same attributes.",
			{
				"<Inheritance>", pInherits, 'ctxt',
				"inherits elements and properties of the text class.",
				reserved, singleItem, notEnumerated, readOnly, Reserved12
			},
			{
			},
			"attribute runs", 'catr', plural__
		},
		{
			/* Comparisons */
		},
		{
			/* Enumerations */
		},

		"NodeBox",
		"NodeBox Script Suite",
		'ndbx',
		1,
		1,
		{
			/* Events */

			"run",
			"Run a document.",
			'ndbx', 'runs',
			reply_none__,
			'docu',
			"The document(s) or window(s) to run.",
			directParamRequired,
			singleItem, notEnumerated, Reserved13,
			{

			},

			"export",
			"Export a document.",
			'ndbx', 'expt',
			reply_none__,
			'docu',
			"The document(s) or window(s) to export.",
			directParamRequired,
			singleItem, notEnumerated, Reserved13,
			{
				"in", 'kfil', 'file',
				"The file in which to save the document",
				required,
				singleItem, notEnumerated, Reserved13,
				"as", 'extp', 'extp',
				"The type of file to export",
				required,
				singleItem, enumerated, Reserved13,
				"pages", 'expg', 'long',
				"The amount of pages to export for PDF documents",
				required,
				singleItem, notEnumerated, Reserved13,
				"frames", 'exfs', 'long',
				"The amount of frames to export for QuickTime movies",
				required,
				singleItem, notEnumerated, Reserved13,
				"framerate", 'exfr', 'long',
				"The framerate for QuickTime movies",
				required,
				singleItem, notEnumerated, Reserved13
			}
		},
		{
			/* Classes */

			"document", 'docu',
			"A NodeBox document",
			{
				"<Inheritance>", pInherits, 'docu',
				"inherits elements and properties of the document class.",
				reserved, singleItem, notEnumerated, readOnly, Reserved12,

				"source", 'psrc', 'ctxt',
				"The source file for this script.",
				reserved, singleItem, notEnumerated, readWrite, Reserved12
			},
			{
			},
			"documents", 'docu', plural__
		},
		{
			/* Comparisons */
		},
		{
			/* Enumerations */
			'extp',
			{
				"QuickTime", 'qtxx', "Export as QuickTime.",
				"PDF", 'pdfx', "Export as PDF."
			}
		}
	}
};
