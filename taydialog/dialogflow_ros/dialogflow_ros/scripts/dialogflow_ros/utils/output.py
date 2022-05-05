# -*- coding: utf-8 -*-
from google.protobuf.struct_pb2 import ListValue

def print_context_parameters(contexts):
    result = []
    for context in contexts:
        param_list = []
        temp_str = '\n\t'
        for parameter in context.parameters:
            if type(context.parameters[parameter]) is ListValue:
                param_list.append("{}: {}".format(
                    parameter, context.parameters[parameter]))
            else:
                parameter = parameter.encode('utf-8')
                param_list.append("{}: {}".format(
                    parameter, context.parameters[parameter]))
        temp_str += "Name: {}\n\tParameters:\n\t {}".format(
                context.name.split('/')[-1], "\n\t".join(param_list))
        result.append(temp_str)
    result = "\n".join(result)
    return result


def print_parameters(parameters):
    param_list = []
    temp_str = '\n\t'
    for parameter in parameters:
        parameter = parameter.encode('utf-8')
        if type(parameters[parameter]) is ListValue or type(parameters[parameter]) is float:
            param_list.append("{}: {}\n\t".format(
                    parameter, parameters[parameter]))
        else:
            param_list.append("{}: {}\n\t".format(
                    parameter, parameters[parameter].encode('utf-8')))
        temp_str += "{}".format("\n\t".join(param_list))
        return temp_str


def print_result(result):
    output = "DF_CLIENT: Results:\n" \
             "Query Text: {}\n" \
             "Detected intent: {} (Confidence: {})\n" \
             "Contexts: {}\n" \
             "Fulfillment text: {}\n" \
             "Action: {}\n" \
             "Parameters: {}".format(
                     result.query_text.encode('utf-8'),
                     result.intent.display_name.encode('utf-8'),
                     result.intent_detection_confidence,
                     print_context_parameters(result.output_contexts),
                     result.fulfillment_text.encode('utf-8'),
                     result.action.encode('utf-8'),
                     print_parameters(result.parameters))
    return output
